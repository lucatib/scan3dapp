using System.Numerics;
using Android.Content;
using Android.Opengl;
using Android.Graphics;
using Android.Views;
using Scanner.App.Controls;

namespace Scanner.App.Droid.Viewer;

/// <summary>GLSurfaceView with one-finger orbit and pinch zoom.</summary>
public sealed class PointCloudGlView : GLSurfaceView
{
    private readonly OrbitRenderer _renderer = new();
    private readonly ScaleGestureDetector _scaleDetector;
    private float _lastX;
    private float _lastY;

    public PointCloudGlView(Context context) : base(context)
    {
        SetEGLContextClientVersion(3);
        SetEGLConfigChooser(8, 8, 8, 8, 16, 0);
        SetRenderer(_renderer);
        RenderMode = Rendermode.WhenDirty;
        _scaleDetector = new ScaleGestureDetector(context, new ScaleListener(this));
    }

    public void SetPoints(Vector3[]? points)
    {
        QueueEvent(() => _renderer.SetPoints(points));
        RequestRender();
    }

    /// <summary>Decodes the photos the model uses off the UI thread, then hands everything to the GL thread.</summary>
    public void SetModel(TexturedScene? scene)
    {
        if (scene is null)
        {
            QueueEvent(() => _renderer.SetModel(null, []));
            RequestRender();
            return;
        }
        Task.Run(() =>
        {
            var pictures = new Dictionary<int, Bitmap>();
            // Half size is 540x960: sharp enough for the patches, and GL-friendly in memory.
            var options = new BitmapFactory.Options { InSampleSize = 2 };
            foreach (int photo in scene.Model.TrianglePhotos.Distinct().Where(p => p > 0))
            {
                string path = System.IO.Path.Combine(scene.Directory, "photos", $"{photo:D6}.jpg");
                if (BitmapFactory.DecodeFile(path, options) is { } picture) pictures[photo] = picture;
            }
            QueueEvent(() => _renderer.SetModel(scene.Model, pictures));
            RequestRender();
        });
    }

    public override bool OnTouchEvent(MotionEvent? e)
    {
        if (e is null) return false;
        _scaleDetector.OnTouchEvent(e);
        if (e.PointerCount == 1 && !_scaleDetector.IsInProgress)
        {
            switch (e.ActionMasked)
            {
                case MotionEventActions.Down:
                    _lastX = e.GetX();
                    _lastY = e.GetY();
                    break;
                case MotionEventActions.Move:
                    float dx = e.GetX() - _lastX, dy = e.GetY() - _lastY;
                    _lastX = e.GetX();
                    _lastY = e.GetY();
                    QueueEvent(() => _renderer.Camera.Rotate(dx, dy));
                    RequestRender();
                    break;
            }
        }
        else if (e.ActionMasked == MotionEventActions.PointerUp && e.PointerCount == 2)
        {
            // Continue rotating smoothly with the finger that stays down.
            int remaining = e.ActionIndex == 0 ? 1 : 0;
            _lastX = e.GetX(remaining);
            _lastY = e.GetY(remaining);
        }
        return true;
    }

    private sealed class ScaleListener(PointCloudGlView owner) : ScaleGestureDetector.SimpleOnScaleGestureListener
    {
        public override bool OnScale(ScaleGestureDetector? detector)
        {
            if (detector is null) return false;
            float factor = detector.ScaleFactor;
            owner.QueueEvent(() => owner._renderer.Camera.Zoom(factor));
            owner.RequestRender();
            return true;
        }
    }
}
