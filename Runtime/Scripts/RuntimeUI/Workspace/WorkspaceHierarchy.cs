using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using UnityEngine.UIElements;

namespace QuadSim.RuntimeUI.Workspace
{
    /// <summary>One selection view over scene assets and drones; their managers retain ownership.</summary>
    public sealed class WorkspaceHierarchy : VisualElement
    {
        public sealed class Entry
        {
            public readonly Transform Target;
            public readonly string Name;
            public Entry(Transform target, string name) { Target = target; Name = name; }
        }
        private const int ObjectsGroup = int.MaxValue, DronesGroup = int.MaxValue - 1;
        public TreeView Tree { get; }
        public event Action<Transform> Selected;
        public event Action<Transform> FrameRequested;
        private readonly List<(int id, int parent, string name)> previous = new();
        private Transform selection;
        private bool initialized;

        public WorkspaceHierarchy()
        {
            AddToClassList("scene-hierarchy");
            Tree = new TreeView { fixedItemHeight = 26, selectionType = SelectionType.Single,
                virtualizationMethod = CollectionVirtualizationMethod.FixedHeight };
            Tree.AddToClassList("entity-list");
            Tree.makeItem = () =>
            {
                var row = new Label(); row.AddToClassList("hierarchy-row");
                // Re-clicking the current row restores explicit highlighting after viewport selection.
                row.RegisterCallback<PointerDownEvent>(e =>
                {
                    if (e.button != 0 || row.userData is not Entry entry) return;
                    Selected?.Invoke(entry.Target);
                    if (e.clickCount == 2 && entry.Target != null) FrameRequested?.Invoke(entry.Target);
                });
                return row;
            };
            Tree.bindItem = (element, index) =>
            {
                var entry = Tree.GetItemDataForIndex<Entry>(index);
                var row = (Label)element;
                row.text = entry.Name; row.userData = entry; row.tooltip = entry.Name;
                row.EnableInClassList("hierarchy-group", entry.Target == null);
            };
            Tree.selectionChanged += items =>
            {
                foreach (Entry entry in items) Selected?.Invoke(entry.Target);
            };
            Add(Tree);
        }

        public void Refresh(IReadOnlyList<Transform> objects, IReadOnlyList<Transform> drones, Transform selected)
        {
            var snapshot = new List<(int, int, string)>();
            foreach (var target in objects.Concat(drones))
                if (target != null) snapshot.Add((target.GetInstanceID(), target.parent != null ? target.parent.GetInstanceID() : 0, target.name));
            if (!initialized || !previous.SequenceEqual(snapshot))
            {
                previous.Clear(); previous.AddRange(snapshot);
                var scene = BuildObjects(objects);
                var aircraft = new List<TreeViewItemData<Entry>>();
                foreach (var target in drones)
                {
                    if (target == null) continue;
                    var pawn = target.GetComponent<DroneCore.QuadPawn>();
                    aircraft.Add(new TreeViewItemData<Entry>(target.GetInstanceID(),
                        new Entry(target, pawn != null ? pawn.DroneID : target.name)));
                }
                Tree.SetRootItems(new List<TreeViewItemData<Entry>>
                {
                    new(ObjectsGroup, new Entry(null, $"Scene Objects ({objects.Count})"), scene),
                    new(DronesGroup, new Entry(null, $"Drones ({aircraft.Count})"), aircraft)
                });
                Tree.Rebuild();
                if (!initialized) { Tree.ExpandRootItems(); initialized = true; }
                selection = null;
            }
            if (selection == selected && (selected != null || Tree.selectedIndex < 0)) return;
            selection = selected;
            Tree.SetSelectionByIdWithoutNotify(selected != null ? new[] { selected.GetInstanceID() } : Array.Empty<int>());
        }

        public static List<TreeViewItemData<Entry>> BuildObjects(IReadOnlyList<Transform> objects)
        {
            var included = new HashSet<Transform>(objects);
            var children = new Dictionary<Transform, List<Transform>>();
            var roots = new List<Transform>();
            foreach (var target in objects)
            {
                if (target == null) continue;
                var parent = target.parent;
                while (parent != null && !included.Contains(parent)) parent = parent.parent;
                if (parent == null) roots.Add(target);
                else
                {
                    if (!children.TryGetValue(parent, out var list)) children[parent] = list = new List<Transform>();
                    list.Add(target);
                }
            }
            TreeViewItemData<Entry> Build(Transform target) => new(target.GetInstanceID(), new Entry(target, target.name),
                children.TryGetValue(target, out var list) ? list.Select(Build).ToList() : null);
            return roots.Select(Build).ToList();
        }
    }
}
