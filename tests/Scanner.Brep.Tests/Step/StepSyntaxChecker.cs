using System.Text.RegularExpressions;

namespace Scanner.Brep.Tests.Step;

/// <summary>Structural checks on a STEP file: header, unique ids, resolved references, terminated lines.</summary>
internal static class StepSyntaxChecker
{
    private static readonly Regex DefinitionPattern = new(@"^#(\d+)=(.+);$", RegexOptions.Multiline);
    private static readonly Regex ReferencePattern = new(@"#(\d+)");

    public static IReadOnlyList<string> Check(string step)
    {
        var errors = new List<string>();
        var text = step.Trim();
        if (!text.StartsWith("ISO-10303-21;", StringComparison.Ordinal)) errors.Add("Missing ISO-10303-21 header.");
        if (!text.EndsWith("END-ISO-10303-21;", StringComparison.Ordinal)) errors.Add("Missing END-ISO-10303-21 closing.");
        if (!text.Contains("FILE_SCHEMA(('AUTOMOTIVE_DESIGN { 1 0 10303 214 1 1 1 1 }'));", StringComparison.Ordinal))
            errors.Add("Missing AP214 schema.");

        var defined = new HashSet<int>();
        foreach (Match m in DefinitionPattern.Matches(step))
            if (!defined.Add(int.Parse(m.Groups[1].Value))) errors.Add($"Duplicate id #{m.Groups[1].Value}.");

        foreach (Match m in ReferencePattern.Matches(step))
            if (!defined.Contains(int.Parse(m.Groups[1].Value))) errors.Add($"Undefined reference #{m.Groups[1].Value}.");

        foreach (var line in step.Split('\n').Where(l => l.StartsWith('#')))
        {
            if (!line.TrimEnd().EndsWith(';')) errors.Add($"Unterminated line: {line}");
            if (line.Count(c => c == '(') != line.Count(c => c == ')')) errors.Add($"Unbalanced parentheses: {line}");
        }
        return errors;
    }

    public static int Count(string step, string entity) => Regex.Matches(step, $@"={entity}\(").Count;

    /// <summary>Instance id to entity text ("NAME(args)", without the trailing semicolon).</summary>
    public static IReadOnlyDictionary<int, string> Entities(string step)
    {
        var entities = new Dictionary<int, string>();
        foreach (Match m in DefinitionPattern.Matches(step))
            entities[int.Parse(m.Groups[1].Value)] = m.Groups[2].Value;
        return entities;
    }

    /// <summary>Entity name of an entity text, i.e. everything before the first parenthesis.</summary>
    public static string Name(string entity) => entity[..entity.IndexOf('(')];

    /// <summary>The instance ids an entity references, in the order they appear in its argument list.</summary>
    public static int[] References(string entity) =>
        ReferencePattern.Matches(entity).Select(m => int.Parse(m.Groups[1].Value)).ToArray();
}
