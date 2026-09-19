using Android.Opengl;
using Google.AR.Core;
using Java.Nio;
using ArFrame = Google.AR.Core.Frame;

namespace Scanner.App.Droid.Rendering;

/// <summary>Draws the ARCore camera image (external OES texture) as a full-screen background.</summary>
internal sealed class CameraBackgroundRenderer
{
    private const int TextureExternalOes = 0x8D65; // GL_TEXTURE_EXTERNAL_OES
    private static readonly float[] QuadCoords = [-1f, -1f, 1f, -1f, -1f, 1f, 1f, 1f];

    private const string VertexShader = """
        #version 300 es
        layout(location = 0) in vec2 a_Position;
        layout(location = 1) in vec2 a_TexCoord;
        out vec2 v_TexCoord;
        void main() {
            gl_Position = vec4(a_Position, 0.0, 1.0);
            v_TexCoord = a_TexCoord;
        }
        """;

    private const string FragmentShader = """
        #version 300 es
        #extension GL_OES_EGL_image_external_essl3 : require
        precision mediump float;
        uniform samplerExternalOES u_Texture;
        in vec2 v_TexCoord;
        out vec4 o_Color;
        void main() {
            o_Color = texture(u_Texture, v_TexCoord);
        }
        """;

    private readonly float[] _texCoords = new float[8];
    private FloatBuffer? _quad;
    private FloatBuffer? _tex;
    private bool _texCoordsValid;
    private int _program;
    private int _textureUniform;

    public int TextureId { get; private set; }

    /// <summary>Creates GL resources. Call on the GL thread from OnSurfaceCreated.</summary>
    public void Initialize()
    {
        var textures = new int[1];
        GLES30.GlGenTextures(1, textures, 0);
        TextureId = textures[0];
        GLES30.GlBindTexture(TextureExternalOes, TextureId);
        GLES30.GlTexParameteri(TextureExternalOes, GLES30.GlTextureWrapS, GLES30.GlClampToEdge);
        GLES30.GlTexParameteri(TextureExternalOes, GLES30.GlTextureWrapT, GLES30.GlClampToEdge);
        GLES30.GlTexParameteri(TextureExternalOes, GLES30.GlTextureMinFilter, GLES30.GlLinear);
        GLES30.GlTexParameteri(TextureExternalOes, GLES30.GlTextureMagFilter, GLES30.GlLinear);

        _program = GlUtil.CreateProgram(VertexShader, FragmentShader);
        _textureUniform = GLES30.GlGetUniformLocation(_program, "u_Texture");
        _quad = GlUtil.ToBuffer(QuadCoords);
        _tex = GlUtil.ToBuffer(new float[8]);
        _texCoordsValid = false; // a new GL context (or a new session) needs the texture coordinates again
    }

    /// <summary>Forces the texture coordinates to be recomputed on the next draw (e.g. a new ARCore session).</summary>
    public void InvalidateTextureCoordinates() => _texCoordsValid = false;

    public void Draw(ArFrame frame)
    {
        if (frame.HasDisplayGeometryChanged || !_texCoordsValid)
        {
            frame.TransformCoordinates2d(Coordinates2d.OpenglNormalizedDeviceCoordinates!, QuadCoords,
                Coordinates2d.TextureNormalized!, _texCoords);
            _tex!.Position(0);
            _tex.Put(_texCoords);
            _tex.Position(0);
            _texCoordsValid = true;
        }
        if (frame.Timestamp == 0) return; // ARCore has not produced a camera image yet.

        GLES30.GlDisable(GLES30.GlDepthTest);
        GLES30.GlDepthMask(false);
        GLES30.GlUseProgram(_program);
        GLES30.GlActiveTexture(GLES30.GlTexture0);
        GLES30.GlBindTexture(TextureExternalOes, TextureId);
        GLES30.GlUniform1i(_textureUniform, 0);
        GLES30.GlVertexAttribPointer(0, 2, GLES30.GlFloat, false, 0, _quad);
        GLES30.GlVertexAttribPointer(1, 2, GLES30.GlFloat, false, 0, _tex);
        GLES30.GlEnableVertexAttribArray(0);
        GLES30.GlEnableVertexAttribArray(1);
        GLES30.GlDrawArrays(GLES30.GlTriangleStrip, 0, 4);
        GLES30.GlDisableVertexAttribArray(0);
        GLES30.GlDisableVertexAttribArray(1);
        GLES30.GlDepthMask(true);
        GLES30.GlEnable(GLES30.GlDepthTest);
    }
}
