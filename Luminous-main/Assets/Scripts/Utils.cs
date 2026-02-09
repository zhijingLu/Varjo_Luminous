using UnityEngine;


public static class Utils
{
    // Converts a string to a colorized version for Unity's UI text components.
    // Debug.Log($"Hello {name.Colorize(Color.cyan)}!");
    // Debug.Log($"Hello {name.Colorize(new Color(0.5f, 0.8f, 0.2f))}!");
    public static string Colorize(this string text, Color color)
    {
        if (string.IsNullOrEmpty(text)) return text;
        string hex = ColorUtility.ToHtmlStringRGB(color); // RRGGBB
        return $"<color=#{hex}>{text}</color>";
    }

    // converts a string to bold version
    public static string Bold(this string text)
    {
        if (string.IsNullOrEmpty(text)) return text;
        return $"<b>{text}</b>";
    }

    // converts a string to italic version
    public static string Italic(this string text)
    {
        if (string.IsNullOrEmpty(text)) return text;
        return $"<i>{text}</i>";
    }
}
