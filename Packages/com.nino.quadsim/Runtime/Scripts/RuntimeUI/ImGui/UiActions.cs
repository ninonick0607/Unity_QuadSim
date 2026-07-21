// Deferred main-thread action queue. ImGui layout callbacks run inside the
// render pipeline — Instantiate/Destroy/physics teleports there crash Unity.
// UI code enqueues mutations; ImGuiRuntimeBootstrap drains in Update().
using System;
using System.Collections.Generic;

namespace QuadSim.RuntimeUI.ImGuiHud
{
    public static class UiActions
    {
        private static readonly Queue<Action> Pending = new();

        public static void Defer(Action action)
        {
            if (action != null) Pending.Enqueue(action);
        }

        public static void Drain()
        {
            while (Pending.Count > 0)
            {
                var a = Pending.Dequeue();
                try { a(); }
                catch (Exception e) { UnityEngine.Debug.LogError($"[UiActions] Deferred action failed: {e}"); }
            }
        }
    }
}