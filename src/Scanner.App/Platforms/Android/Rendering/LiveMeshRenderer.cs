using Android.Opengl;
using Scanner.Core.Meshing;

namespace Scanner.App.Droid.Rendering;

/// <summary>Draws the live photo reconstruction over the camera image as a lit, clay-coloured surface. GL thread only.</summary>
internal sealed class LiveMeshRenderer
{
    private const string VertexShader = """
        #version 300 es
        uniform mat4 u_ViewProjection;
        layout(location = 0) in vec3 a_Position;
        layout(location = 1) in vec3 a_Normal;
        out vec3 v_Normal;
        void main() {
            gl_Position = u_ViewProjection * vec4(a_Position, 1.0);
            v_Normal = a_Normal;
        }
        """;

    private const string FragmentShader = """
        #version 300 es
        precision mediump float;
        in vec3 v_Normal;
        out vec4 o_Color;
        void main() {
            vec3 n = normalize(v_Normal);
            float key = max(dot(n, normalize(vec3(0.3, 1.0, 0.4))), 0.0);
            float fill = max(dot(n, normalize(vec3(-0.6, 0.4, -0.7))), 0.0);
            vec3 clay = vec3(0.93, 0.80, 0.62);
            o_Color = vec4(clay * (0.30 + 0.65 * key + 0.25 * fill), 1.0);
        }
        """;

    private int _program;
    private int _viewProjectionUniform;
    private int _vertexBuffer;
    private int _indexBuffer;
    private int _indexCount;

    public bool HasMesh => _indexCount > 0;

    public void Initialize()
    {
        _program = GlUtil.CreateProgram(VertexShader, FragmentShader);
        _viewProjectionUniform = GLES30.GlGetUniformLocation(_program, "u_ViewProjection");
        var buffers = new int[2];
        GLES30.GlGenBuffers(2, buffers, 0);
        _vertexBuffer = buffers[0];
        _indexBuffer = buffers[1];
        _indexCount = 0;
    }

    /// <param name="transform">Applied to every vertex (row vectors): the live surface is built with refined poses and
    /// is drawn in ARCore's current frame.</param>
    public void Upload(TriangleMesh? mesh, System.Numerics.Matrix4x4 transform)
    {
        if (mesh is null || mesh.TriangleCount == 0)
        {
            _indexCount = 0;
            return;
        }
        var data = new float[mesh.Positions.Count * 6];
        for (int i = 0; i < mesh.Positions.Count; i++)
        {
            var p = System.Numerics.Vector3.Transform(mesh.Positions[i], transform);
            var n = System.Numerics.Vector3.TransformNormal(mesh.Normals[i], transform);
            data[6 * i] = p.X; data[6 * i + 1] = p.Y; data[6 * i + 2] = p.Z;
            data[6 * i + 3] = n.X; data[6 * i + 4] = n.Y; data[6 * i + 5] = n.Z;
        }
        var indices = mesh.Indices.ToArray();
        var indexBuffer = Java.Nio.ByteBuffer.AllocateDirect(indices.Length * sizeof(int))
            .Order(Java.Nio.ByteOrder.NativeOrder()!)!.AsIntBuffer()!;
        indexBuffer.Put(indices);
        indexBuffer.Position(0);

        GLES30.GlBindBuffer(GLES30.GlArrayBuffer, _vertexBuffer);
        GLES30.GlBufferData(GLES30.GlArrayBuffer, data.Length * sizeof(float), GlUtil.ToBuffer(data), GLES30.GlDynamicDraw);
        GLES30.GlBindBuffer(GLES30.GlArrayBuffer, 0);
        GLES30.GlBindBuffer(GLES30.GlElementArrayBuffer, _indexBuffer);
        GLES30.GlBufferData(GLES30.GlElementArrayBuffer, indices.Length * sizeof(int), indexBuffer, GLES30.GlDynamicDraw);
        GLES30.GlBindBuffer(GLES30.GlElementArrayBuffer, 0);
        _indexCount = indices.Length;
    }

    /// <param name="viewProjection">Column-major 4x4 matrix (GL convention).</param>
    public void Draw(float[] viewProjection)
    {
        if (_indexCount == 0) return;
        GLES30.GlEnable(GLES30.GlDepthTest);
        GLES30.GlUseProgram(_program);
        GLES30.GlUniformMatrix4fv(_viewProjectionUniform, 1, false, viewProjection, 0);
        GLES30.GlBindBuffer(GLES30.GlArrayBuffer, _vertexBuffer);
        GLES30.GlVertexAttribPointer(0, 3, GLES30.GlFloat, false, 6 * sizeof(float), 0);
        GLES30.GlVertexAttribPointer(1, 3, GLES30.GlFloat, false, 6 * sizeof(float), 3 * sizeof(float));
        GLES30.GlEnableVertexAttribArray(0);
        GLES30.GlEnableVertexAttribArray(1);
        GLES30.GlBindBuffer(GLES30.GlElementArrayBuffer, _indexBuffer);
        GLES30.GlDrawElements(GLES30.GlTriangles, _indexCount, GLES30.GlUnsignedInt, 0);
        GLES30.GlDisableVertexAttribArray(0);
        GLES30.GlDisableVertexAttribArray(1);
        GLES30.GlBindBuffer(GLES30.GlArrayBuffer, 0);
        GLES30.GlBindBuffer(GLES30.GlElementArrayBuffer, 0);
        GLES30.GlDisable(GLES30.GlDepthTest);
    }
}
