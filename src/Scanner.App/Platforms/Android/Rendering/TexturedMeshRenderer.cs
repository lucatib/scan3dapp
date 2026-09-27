using Android.Graphics;
using Android.Opengl;
using Scanner.Capture.Sessions;

namespace Scanner.App.Droid.Rendering;

/// <summary>
/// Draws a <see cref="TexturedModel"/>: one vertex buffer and one texture per photo, each triangle showing the patch of
/// the photo it was cut from. Untextured triangles are grey. All calls on the GL thread.
/// </summary>
internal sealed class TexturedMeshRenderer
{
    private const string VertexShader = """
        #version 300 es
        uniform mat4 u_ViewProjection;
        layout(location = 0) in vec3 a_Position;
        layout(location = 1) in vec2 a_Uv;
        out vec2 v_Uv;
        void main() {
            gl_Position = u_ViewProjection * vec4(a_Position, 1.0);
            v_Uv = a_Uv;
        }
        """;

    private const string FragmentShader = """
        #version 300 es
        precision mediump float;
        uniform sampler2D u_Texture;
        uniform bool u_Textured;
        in vec2 v_Uv;
        out vec4 o_Color;
        void main() {
            o_Color = u_Textured ? vec4(texture(u_Texture, v_Uv).rgb, 1.0) : vec4(0.6, 0.6, 0.6, 1.0);
        }
        """;

    private int _program;
    private int _viewProjectionUniform;
    private int _texturedUniform;
    private int _textureUniform;
    private readonly List<Group> _groups = [];

    private sealed record Group(int Buffer, int VertexCount, int Texture);

    public bool HasModel => _groups.Count > 0;

    public void Initialize()
    {
        _program = GlUtil.CreateProgram(VertexShader, FragmentShader);
        _viewProjectionUniform = GLES30.GlGetUniformLocation(_program, "u_ViewProjection");
        _texturedUniform = GLES30.GlGetUniformLocation(_program, "u_Textured");
        _textureUniform = GLES30.GlGetUniformLocation(_program, "u_Texture");
        _groups.Clear(); // a new EGL context has none of the old names
    }

    /// <summary>Uploads the model; <paramref name="pictures"/> maps photo index to its decoded picture. The caller keeps
    /// the pictures: a recreated EGL context needs them uploaded again.</summary>
    public void Upload(TexturedModel? model, IReadOnlyDictionary<int, Bitmap> pictures)
    {
        Release();
        if (model is null) return;
        foreach (var triangles in Enumerable.Range(0, model.TriangleCount).GroupBy(t => model.TrianglePhotos[t]))
        {
            var data = new List<float>();
            foreach (int t in triangles)
            for (int k = 0; k < 3; k++)
            {
                var p = model.Positions[model.Indices[3 * t + k]];
                var uv = model.Uvs[3 * t + k];
                data.AddRange([p.X, p.Y, p.Z, uv.X, uv.Y]);
            }
            var buffers = new int[1];
            GLES30.GlGenBuffers(1, buffers, 0);
            GLES30.GlBindBuffer(GLES30.GlArrayBuffer, buffers[0]);
            GLES30.GlBufferData(GLES30.GlArrayBuffer, data.Count * sizeof(float), GlUtil.ToBuffer(data.ToArray()), GLES30.GlStaticDraw);
            GLES30.GlBindBuffer(GLES30.GlArrayBuffer, 0);

            int texture = 0;
            if (triangles.Key > 0 && pictures.TryGetValue(triangles.Key, out var picture))
            {
                var textures = new int[1];
                GLES30.GlGenTextures(1, textures, 0);
                texture = textures[0];
                GLES30.GlBindTexture(GLES30.GlTexture2d, texture);
                GLES30.GlTexParameteri(GLES30.GlTexture2d, GLES30.GlTextureMinFilter, GLES30.GlLinearMipmapLinear);
                GLES30.GlTexParameteri(GLES30.GlTexture2d, GLES30.GlTextureMagFilter, GLES30.GlLinear);
                GLES30.GlTexParameteri(GLES30.GlTexture2d, GLES30.GlTextureWrapS, GLES30.GlClampToEdge);
                GLES30.GlTexParameteri(GLES30.GlTexture2d, GLES30.GlTextureWrapT, GLES30.GlClampToEdge);
                GLUtils.TexImage2D(GLES30.GlTexture2d, 0, picture, 0);
                GLES30.GlGenerateMipmap(GLES30.GlTexture2d);
                GLES30.GlBindTexture(GLES30.GlTexture2d, 0);
            }
            _groups.Add(new Group(buffers[0], data.Count / 5, texture));
        }
    }

    /// <param name="viewProjection">Column-major 4x4 matrix (GL convention).</param>
    public void Draw(float[] viewProjection)
    {
        if (_groups.Count == 0) return;
        GLES30.GlUseProgram(_program);
        GLES30.GlUniformMatrix4fv(_viewProjectionUniform, 1, false, viewProjection, 0);
        GLES30.GlUniform1i(_textureUniform, 0);
        GLES30.GlActiveTexture(GLES30.GlTexture0);
        GLES30.GlEnableVertexAttribArray(0);
        GLES30.GlEnableVertexAttribArray(1);
        foreach (var group in _groups)
        {
            GLES30.GlUniform1i(_texturedUniform, group.Texture != 0 ? 1 : 0);
            GLES30.GlBindTexture(GLES30.GlTexture2d, group.Texture);
            GLES30.GlBindBuffer(GLES30.GlArrayBuffer, group.Buffer);
            GLES30.GlVertexAttribPointer(0, 3, GLES30.GlFloat, false, 5 * sizeof(float), 0);
            GLES30.GlVertexAttribPointer(1, 2, GLES30.GlFloat, false, 5 * sizeof(float), 3 * sizeof(float));
            GLES30.GlDrawArrays(GLES30.GlTriangles, 0, group.VertexCount);
        }
        GLES30.GlDisableVertexAttribArray(0);
        GLES30.GlDisableVertexAttribArray(1);
        GLES30.GlBindBuffer(GLES30.GlArrayBuffer, 0);
        GLES30.GlBindTexture(GLES30.GlTexture2d, 0);
    }

    private void Release()
    {
        foreach (var group in _groups)
        {
            GLES30.GlDeleteBuffers(1, [group.Buffer], 0);
            if (group.Texture != 0) GLES30.GlDeleteTextures(1, [group.Texture], 0);
        }
        _groups.Clear();
    }
}
