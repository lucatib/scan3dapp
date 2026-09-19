using System.Numerics;
using Android.Opengl;
using Java.Nio;

namespace Scanner.App.Droid.Rendering;

/// <summary>Draws a point cloud as round GL points coloured by height (world +Y). All calls on the GL thread.</summary>
internal sealed class PointCloudRenderer
{
    private const string VertexShader = """
        #version 300 es
        uniform mat4 u_ViewProjection;
        uniform float u_PointSize;
        uniform vec2 u_HeightRange;
        layout(location = 0) in vec3 a_Position;
        out vec3 v_Color;
        vec3 heightColor(float t) {
            return clamp(vec3(1.5 - abs(4.0 * t - 3.0), 1.5 - abs(4.0 * t - 2.0), 1.5 - abs(4.0 * t - 1.0)), 0.0, 1.0);
        }
        void main() {
            gl_Position = u_ViewProjection * vec4(a_Position, 1.0);
            gl_PointSize = u_PointSize;
            float span = max(u_HeightRange.y - u_HeightRange.x, 1e-4);
            v_Color = heightColor(clamp((a_Position.y - u_HeightRange.x) / span, 0.0, 1.0));
        }
        """;

    private const string FragmentShader = """
        #version 300 es
        precision mediump float;
        in vec3 v_Color;
        out vec4 o_Color;
        void main() {
            vec2 c = gl_PointCoord * 2.0 - 1.0;
            if (dot(c, c) > 1.0) discard;
            o_Color = vec4(v_Color, 1.0);
        }
        """;

    private int _program;
    private int _buffer;
    private int _count;
    private int _viewProjectionUniform;
    private int _pointSizeUniform;
    private int _heightRangeUniform;
    private float _minY;
    private float _maxY;
    private float[] _data = [];
    private FloatBuffer? _staging;

    public void Initialize()
    {
        _program = GlUtil.CreateProgram(VertexShader, FragmentShader);
        _viewProjectionUniform = GLES30.GlGetUniformLocation(_program, "u_ViewProjection");
        _pointSizeUniform = GLES30.GlGetUniformLocation(_program, "u_PointSize");
        _heightRangeUniform = GLES30.GlGetUniformLocation(_program, "u_HeightRange");
        var buffers = new int[1];
        GLES30.GlGenBuffers(1, buffers, 0);
        _buffer = buffers[0];
        _count = 0;
    }

    public void Upload(IReadOnlyList<Vector3> points)
    {
        int length = points.Count * 3;
        if (_data.Length < length || _staging is null || _staging.Capacity() < length)
        {
            int capacity = Math.Max(length + length / 2, 3 * 1024); // grow geometrically: uploads repeat several times a second
            _data = new float[capacity];
            _staging = GlUtil.AllocateFloats(capacity);
        }

        float minY = float.PositiveInfinity, maxY = float.NegativeInfinity;
        for (int i = 0; i < points.Count; i++)
        {
            var p = points[i];
            _data[3 * i] = p.X;
            _data[3 * i + 1] = p.Y;
            _data[3 * i + 2] = p.Z;
            minY = MathF.Min(minY, p.Y);
            maxY = MathF.Max(maxY, p.Y);
        }
        _staging.Clear();
        _staging.Put(_data, 0, length);
        _staging.Position(0);

        GLES30.GlBindBuffer(GLES30.GlArrayBuffer, _buffer);
        GLES30.GlBufferData(GLES30.GlArrayBuffer, length * sizeof(float), _staging, GLES30.GlDynamicDraw);
        GLES30.GlBindBuffer(GLES30.GlArrayBuffer, 0);
        _count = points.Count;
        _minY = points.Count > 0 ? minY : 0;
        _maxY = points.Count > 0 ? maxY : 1;
    }

    /// <param name="viewProjection">Column-major 4x4 matrix (GL convention).</param>
    public void Draw(float[] viewProjection, float pointSize)
    {
        if (_count == 0) return;
        GLES30.GlUseProgram(_program);
        GLES30.GlUniformMatrix4fv(_viewProjectionUniform, 1, false, viewProjection, 0);
        GLES30.GlUniform1f(_pointSizeUniform, pointSize);
        GLES30.GlUniform2f(_heightRangeUniform, _minY, _maxY);
        GLES30.GlBindBuffer(GLES30.GlArrayBuffer, _buffer);
        GLES30.GlVertexAttribPointer(0, 3, GLES30.GlFloat, false, 0, 0);
        GLES30.GlEnableVertexAttribArray(0);
        GLES30.GlDrawArrays(GLES30.GlPoints, 0, _count);
        GLES30.GlDisableVertexAttribArray(0);
        GLES30.GlBindBuffer(GLES30.GlArrayBuffer, 0);
    }
}
