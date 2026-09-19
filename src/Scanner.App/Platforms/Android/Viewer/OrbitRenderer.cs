using System.Numerics;
using Android.Opengl;
using Javax.Microedition.Khronos.Opengles;
using Scanner.App.Controls;
using Scanner.App.Droid.Rendering;
using EGLConfig = Javax.Microedition.Khronos.Egl.EGLConfig;

namespace Scanner.App.Droid.Viewer;

/// <summary>GL-thread renderer for the 3D preview. Mutate <see cref="Camera"/> only through GLSurfaceView.QueueEvent.</summary>
internal sealed class OrbitRenderer : Java.Lang.Object, GLSurfaceView.IRenderer
{
    private const float PointSizePixels = 6f;

    private readonly PointCloudRenderer _points = new();
    private Vector3[] _data = [];
    private float _aspect = 1f;

    public OrbitCamera Camera { get; } = new();

    /// <summary>GL thread.</summary>
    public void SetPoints(Vector3[]? points)
    {
        _data = points ?? [];
        _points.Upload(_data);
        Camera.Frame(_data);
    }

    public void OnSurfaceCreated(IGL10? gl, EGLConfig? config)
    {
        GLES30.GlClearColor(0.11f, 0.11f, 0.13f, 1f);
        GLES30.GlEnable(GLES30.GlDepthTest);
        _points.Initialize();
        _points.Upload(_data); // the EGL context may have been recreated
    }

    public void OnSurfaceChanged(IGL10? gl, int width, int height)
    {
        GLES30.GlViewport(0, 0, width, height);
        _aspect = height > 0 ? (float)width / height : 1f;
    }

    public void OnDrawFrame(IGL10? gl)
    {
        GLES30.GlClear(GLES30.GlColorBufferBit | GLES30.GlDepthBufferBit);
        _points.Draw(Camera.ViewProjection(_aspect), PointSizePixels);
    }
}
