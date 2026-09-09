using System;
using System.Collections.Generic;
using UnityEngine.InputSystem;

namespace QuadSim.RuntimeUI.Workspace
{
    /// <summary>Remove camera keys from UI navigation for this session, preserving user bindings on disposal.</summary>
    public sealed class WorkspaceNavigationBindings : IDisposable
    {
        private readonly List<(InputAction Action, int Index, InputBinding Original)> overrides = new();
        public WorkspaceNavigationBindings()
        {
            InputSystem.onActionChange += OnActionChange;
            foreach (var action in InputSystem.ListEnabledActions()) Restrict(action);
        }
        private void OnActionChange(object source, InputActionChange change)
        {
            if (change == InputActionChange.ActionEnabled && source is InputAction action) Restrict(action);
            if (change == InputActionChange.ActionMapEnabled && source is InputActionMap map)
                foreach (var item in map.actions) Restrict(item);
        }
        private void Restrict(InputAction action)
        {
            if (action.name != "Navigate" || action.actionMap?.name != "UI") return;
            for (int i = 0; i < action.bindings.Count; i++)
            {
                var binding = action.bindings[i];
                string path = binding.effectivePath?.ToLowerInvariant();
                if (path != "<keyboard>/w" && path != "<keyboard>/a" && path != "<keyboard>/s" && path != "<keyboard>/d") continue;
                overrides.Add((action, i, binding));
                var replacement = binding; replacement.overridePath = "";
                action.ApplyBindingOverride(i, replacement);
            }
        }
        public void Dispose()
        {
            InputSystem.onActionChange -= OnActionChange;
            foreach (var item in overrides) item.Action.ApplyBindingOverride(item.Index, item.Original);
            overrides.Clear();
        }
    }
}
