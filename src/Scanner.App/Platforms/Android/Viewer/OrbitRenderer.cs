using System.Numerics;
using Android.Graphics;
using Android.Opengl;
using Javax.Microedition.Khronos.Opengles;
using Scanner.App.Controls;
using Scanner.App.Droid.Rendering;
using Scanner.Capture.Sessions;
using EGLConfig = Javax.Microedition.Khronos.Egl.EGLConfig;

namespace Scanner.App.Droid.Viewer;

/// <summary>GL-thread renderer for the 3D preview: the textured model when there is one, the point cloud otherwise.
/// Mutate <see cref="Camera"/> only through GLSurfaceView.QueueEvent.</summary>
internal sealed class OrbitRenderer : Java.Lang.Object, GLSurfaceView.IRenderer
{
    private const float PointSizePixels = 6f;

    private readonly PointCloudRenderer _points = new();
    private readonly TexturedMeshRenderer _mesh = new();
    private Vector3[] _data = [];
    private TexturedModel? _model;
    private Dictionary<int, Bitmap> _pictures = [];
    private float _aspect = 1f;

    public OrbitCamera Camera { get; } = new();

    /// <summary>GL thread.</summary>
    public void SetPoints(Vector3[]? points)
    {
        _data = points ?? [];
        _points.Upload(_data);
        if (_model is null) Camera.Frame(_data);
    }

    /// <summary>GL thread. Takes ownership of <paramref name="pictures"/>.</summary>
    public void SetModel(TexturedModel? model, Dictionary<int, Bitmap> pictures)
    {
        foreach (var picture in _pictures.Values) picture.Recycle();
        _model = model;
        _pictures = pictures;
        _mesh.Upload(model, pictures);
        if (model is not null) Camera.Frame(model.Positions);
    }

    public void OnSurfaceCreated(IGL10? gl, EGLConfig? config)
    {
        GLES30.GlClearColor(0.11f, 0.11f, 0.13f, 1f);
        GLES30.GlEnable(GLES30.GlDepthTest);
        _points.Initialize();
        _mesh.Initialize();
        _points.Upload(_data); // the EGL context may have been recreated
        _mesh.Upload(_model, _pictures);
    }

    public void OnSurfaceChanged(IGL10? gl, int width, int height)
    {
        GLES30.GlViewport(0, 0, width, height);
        _aspect = height > 0 ? (float)width / height : 1f;
    }

    public void OnDrawFrame(IGL10? gl)
    {
        GLES30.GlClear(GLES30.GlColorBufferBit | GLES30.GlDepthBufferBit);
        var viewProjection = Camera.ViewProjection(_aspect);
        if (_mesh.HasModel) _mesh.Draw(viewProjection);
        else _points.Draw(viewProjection, PointSizePixels);
    }
}
