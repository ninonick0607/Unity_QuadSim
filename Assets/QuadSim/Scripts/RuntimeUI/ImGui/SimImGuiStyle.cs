// Port of FSimImGuiStyle. Pure ImGuiNET — no binding-specific code.
using ImGuiNET;
using ImVec2 = System.Numerics.Vector2;
using ImVec4 = System.Numerics.Vector4;

namespace QuadSim.RuntimeUI.ImGuiHud
{
    public static class SimImGuiStyle
    {
        // Base palette (dark theme)
        public static readonly ImVec4 WindowBg     = new(0.09f, 0.10f, 0.12f, 0.98f);
        public static readonly ImVec4 ChildBg      = new(0.11f, 0.12f, 0.15f, 0.98f);
        public static readonly ImVec4 PopupBg      = new(0.08f, 0.09f, 0.11f, 0.98f);
        public static readonly ImVec4 Border       = new(0.20f, 0.22f, 0.26f, 1.00f);
        public static readonly ImVec4 Text         = new(0.93f, 0.96f, 1.00f, 1.00f);

        public static readonly ImVec4 FrameBg      = new(0.16f, 0.18f, 0.22f, 1.00f);
        public static readonly ImVec4 FrameHovered = new(0.22f, 0.24f, 0.28f, 1.00f);
        public static readonly ImVec4 FrameActive  = new(0.28f, 0.31f, 0.36f, 1.00f);

        public static readonly ImVec4 Button       = new(0.18f, 0.24f, 0.35f, 1.00f);
        public static readonly ImVec4 ButtonHover  = new(0.24f, 0.32f, 0.48f, 1.00f);
        public static readonly ImVec4 ButtonActive = new(0.16f, 0.28f, 0.46f, 1.00f);

        // Accents
        public static readonly ImVec4 AccentGreen  = new(0.20f, 0.80f, 0.35f, 1.00f);
        public static readonly ImVec4 AccentRed    = new(0.90f, 0.25f, 0.30f, 1.00f);
        public static readonly ImVec4 HeaderBlue   = new(0.60f, 0.90f, 1.00f, 1.00f);
        public static readonly ImVec4 HeaderYellow = new(1.00f, 0.90f, 0.40f, 1.00f);
        public static readonly ImVec4 HeaderPurple = new(0.80f, 0.80f, 1.00f, 1.00f);
        public static readonly ImVec4 WarnOrange   = new(0.95f, 0.60f, 0.15f, 1.00f);

        public static void Apply()
        {
            var style = ImGui.GetStyle();
            var col = style.Colors;

            col[(int)ImGuiCol.Text]           = Text;
            col[(int)ImGuiCol.WindowBg]       = WindowBg;
            col[(int)ImGuiCol.ChildBg]        = ChildBg;
            col[(int)ImGuiCol.PopupBg]        = PopupBg;
            col[(int)ImGuiCol.Border]         = Border;
            col[(int)ImGuiCol.FrameBg]        = FrameBg;
            col[(int)ImGuiCol.FrameBgHovered] = FrameHovered;
            col[(int)ImGuiCol.FrameBgActive]  = FrameActive;
            col[(int)ImGuiCol.Button]         = Button;
            col[(int)ImGuiCol.ButtonHovered]  = ButtonHover;
            col[(int)ImGuiCol.ButtonActive]   = ButtonActive;

            style.FrameRounding    = 5f;
            style.GrabRounding     = 5f;
            style.WindowBorderSize = 1f;
            style.FrameBorderSize  = 0f;
            style.ItemSpacing      = new ImVec2(7f, 5f);
            style.FramePadding     = new ImVec2(7f, 4f);
        }

        public static uint Col32(byte r, byte g, byte b, byte a)
            => (uint)((a << 24) | (b << 16) | (g << 8) | r);
    }

    /// <summary>Shared widget helpers (rounded label boxes, slider+input pattern).</summary>
    internal static class ImGuiWidgets
    {
        public static bool RoundedLabel(string id, string text, float w, float h, bool activeTint = false)
        {
            ImGui.InvisibleButton(id, new ImVec2(w, h));
            var min = ImGui.GetItemRectMin();
            var max = ImGui.GetItemRectMax();
            var dl = ImGui.GetWindowDrawList();
            dl.AddRectFilled(min, max, ImGui.GetColorU32(SimImGuiStyle.ChildBg), 8f);
            dl.AddRect(min, max, ImGui.GetColorU32(SimImGuiStyle.Border), 8f, ImDrawFlags.None, 1f);
            var ts = ImGui.CalcTextSize(text);
            var tp = new ImVec2(min.X + (w - ts.X) * 0.5f, min.Y + (h - ts.Y) * 0.5f);
            uint tc = activeTint ? SimImGuiStyle.Col32(51, 230, 77, 255)
                                 : ImGui.GetColorU32(ImGuiCol.Text);
            dl.AddText(tp, tc, text);
            return ImGui.IsItemClicked();
        }

        /// <summary>Slider + editable input box (old DrawPIDGainControl pattern).</summary>
        public static bool SliderWithInput(string label, ref float v, float min, float max, string fmt = "%.4f")
        {
            float total = ImGui.GetContentRegionAvail().X;
            const float inputW = 80f;
            float sliderW = total > inputW + 20f ? total - inputW - 20f : 100f;

            bool changed = false;
            ImGui.PushItemWidth(sliderW);
            changed |= ImGui.SliderFloat(label, ref v, min, max, fmt);
            ImGui.PopItemWidth();
            ImGui.SameLine();
            ImGui.PushItemWidth(inputW);
            changed |= ImGui.InputFloat("##in_" + label, ref v, 0f, 0f, fmt);
            ImGui.PopItemWidth();
            return changed;
        }
    }
}