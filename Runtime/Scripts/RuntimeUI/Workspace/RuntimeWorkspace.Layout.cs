using System;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UIElements;

namespace QuadSim.RuntimeUI.Workspace
{
    public sealed partial class RuntimeWorkspace
    {
        [Serializable] private sealed class SavedWindow
        {
            public string id; public int dock; public bool open, expanded, active;
            public Vector2 position, size;
        }
        [Serializable] private sealed class SavedWorkspace
        {
            public int version = 1; public float leftWidth, rightWidth, bottomHeight;
            public List<SavedWindow> windows = new List<SavedWindow>();
        }
        private void SetDarkTheme(bool dark)
        {
            if (root == null) return;
            if (darkStyle != null)
            {
                if (dark && !root.styleSheets.Contains(darkStyle)) root.styleSheets.Add(darkStyle);
                if (!dark) root.styleSheets.Remove(darkStyle);
            }
            settings.themeStyleSheet = Resources.Load<ThemeStyleSheet>(dark ? "QuadSimWorkspaceDarkTheme" : "QuadSimWorkspaceTheme");
            PlayerPrefs.SetInt("QuadSim.UI.DarkTheme", dark ? 1 : 0);
            PlayerPrefs.Save();
        }
        private void SaveWorkspace()
        {
            if (root == null || restoringLayout || windows.Count == 0) return;
            var saved = new SavedWorkspace { leftWidth = left.resolvedStyle.width, rightWidth = right.resolvedStyle.width, bottomHeight = bottom.resolvedStyle.height };
            foreach (var w in windows.Values)
                saved.windows.Add(new SavedWindow { id = w.Id, dock = (int)w.Dock, open = w.IsOpen, expanded = w.IsExpanded,
                    active = w.IsPresented, position = w.FloatingPosition, size = w.FloatingSize });
            PlayerPrefs.SetString("QuadSim.UI.Workspace.v1", JsonUtility.ToJson(saved)); PlayerPrefs.Save();
        }
        private void RestoreWorkspace()
        {
            restoringLayout = true;
            try
            {
                foreach (var w in windows.Values)
                {
                    w.Place(w.Id == "Scene" ? WorkspaceDock.Left : w.Id == "Logs" ? WorkspaceDock.Bottom : WorkspaceDock.Right);
                    w.Close();
                }
                var json = PlayerPrefs.GetString("QuadSim.UI.Workspace.v1", "");
                if (string.IsNullOrEmpty(json)) { windows["Scene"].Open(); windows["Inspector"].Open(); return; }
                var saved = JsonUtility.FromJson<SavedWorkspace>(json);
                if (saved == null || saved.version != 1 || saved.windows == null) return;
                if (saved.leftWidth > 0) left.style.width = Mathf.Clamp(saved.leftWidth, 160, 650);
                if (saved.rightWidth > 0) right.style.width = Mathf.Clamp(saved.rightWidth, 160, 650);
                if (saved.bottomHeight > 0) bottom.style.height = Mathf.Clamp(saved.bottomHeight, 100, 600);
                foreach (var item in saved.windows)
                {
                    if (!windows.TryGetValue(item.id, out var w) || !Enum.IsDefined(typeof(WorkspaceDock), item.dock)) continue;
                    w.Restore((WorkspaceDock)item.dock, item.position, item.size);
                    if (w.IsExpanded != item.expanded) w.ToggleExpanded();
                    if (!item.open) w.Close();
                }
                foreach (var item in saved.windows)
                    if (item.active && item.open && windows.TryGetValue(item.id, out var w)) w.Open();
            }
            catch (Exception e) { Debug.LogWarning("[Workspace] Layout could not be restored: " + e.Message); }
            finally { restoringLayout = false; RefreshDocks(); }
        }
        private void BuildDockResizers()
        {
            AddDockResizer(left, WorkspaceDock.Left);
            AddDockResizer(right, WorkspaceDock.Right);
            AddDockResizer(bottom, WorkspaceDock.Bottom);
            docking.RegionTarget = world =>
            {
                if (left.resolvedStyle.display != DisplayStyle.None && left.worldBound.Contains(world)) return WorkspaceDock.Left;
                if (right.resolvedStyle.display != DisplayStyle.None && right.worldBound.Contains(world)) return WorkspaceDock.Right;
                if (bottom.resolvedStyle.display != DisplayStyle.None && bottom.worldBound.Contains(world)) return WorkspaceDock.Bottom;
                return WorkspaceDock.Floating;
            };
        }
        private void AddDockResizer(VisualElement region, WorkspaceDock dock)
        {
            var handle = new VisualElement(); handle.AddToClassList("dock-resizer");
            handle.AddToClassList(dock == WorkspaceDock.Bottom ? "horizontal" : dock == WorkspaceDock.Left ? "right-edge" : "left-edge");
            region.Add(handle); int pointer = -1; Vector2 start = default; float size = 0;
            handle.RegisterCallback<PointerDownEvent>(e => { if (e.button != 0) return; pointer = e.pointerId; start = e.position; size = dock == WorkspaceDock.Bottom ? region.layout.height : region.layout.width; handle.CapturePointer(pointer); e.StopPropagation(); });
            handle.RegisterCallback<PointerMoveEvent>(e =>
            {
                if (pointer != e.pointerId) return;
                Vector2 delta = (Vector2)e.position - start;
                if (dock == WorkspaceDock.Bottom) region.style.height = Mathf.Clamp(size - delta.y, 100, root.layout.height * 0.65f);
                else region.style.width = Mathf.Clamp(size + (dock == WorkspaceDock.Left ? delta.x : -delta.x), 160, root.layout.width * 0.42f);
                e.StopPropagation();
            });
            handle.RegisterCallback<PointerUpEvent>(e => { if (pointer != e.pointerId) return; pointer = -1; handle.ReleasePointer(e.pointerId); SaveWorkspace(); e.StopPropagation(); });
            handle.RegisterCallback<PointerCaptureOutEvent>(_ => pointer = -1);
        }
    }
}
