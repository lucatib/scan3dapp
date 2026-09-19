using Android.Opengl;
using Java.Nio;

namespace Scanner.App.Droid.Rendering;

internal static class GlUtil
{
    public static int CreateProgram(string vertexSource, string fragmentSource)
    {
        int vertex = Compile(GLES30.GlVertexShader, vertexSource);
        int fragment = Compile(GLES30.GlFragmentShader, fragmentSource);
        int program = GLES30.GlCreateProgram();
        GLES30.GlAttachShader(program, vertex);
        GLES30.GlAttachShader(program, fragment);
        GLES30.GlLinkProgram(program);
        var status = new int[1];
        GLES30.GlGetProgramiv(program, GLES30.GlLinkStatus, status, 0);
        GLES30.GlDeleteShader(vertex);
        GLES30.GlDeleteShader(fragment);
        if (status[0] == 0)
        {
            string log = GLES30.GlGetProgramInfoLog(program) ?? "";
            GLES30.GlDeleteProgram(program);
            throw new InvalidOperationException($"GL program link failed: {log}");
        }
        return program;
    }

    public static FloatBuffer ToBuffer(float[] data)
    {
        var floats = AllocateFloats(data.Length);
        floats.Put(data);
        floats.Position(0);
        return floats;
    }

    /// <summary>Allocates a direct, native-order float buffer of the given capacity.</summary>
    public static FloatBuffer AllocateFloats(int capacity)
    {
        var bytes = ByteBuffer.AllocateDirect(capacity * sizeof(float))!;
        bytes.Order(ByteOrder.NativeOrder()!);
        return bytes.AsFloatBuffer()!;
    }

    private static int Compile(int type, string source)
    {
        int shader = GLES30.GlCreateShader(type);
        GLES30.GlShaderSource(shader, source);
        GLES30.GlCompileShader(shader);
        var status = new int[1];
        GLES30.GlGetShaderiv(shader, GLES30.GlCompileStatus, status, 0);
        if (status[0] == 0)
        {
            string log = GLES30.GlGetShaderInfoLog(shader) ?? "";
            GLES30.GlDeleteShader(shader);
            throw new InvalidOperationException($"GL shader compile failed: {log}");
        }
        return shader;
    }
}
