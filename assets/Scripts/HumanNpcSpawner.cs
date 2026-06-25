using Godot;
using System;
using System.Collections.Generic;

/// <summary>
/// Spawns and recycles a pool of <see cref="MobileNpc"/> pedestrians and sends
/// them wandering to random points on the navigation meshes available in the
/// owning <see cref="HillChunk"/>.
///
/// The spawner is self driving: drop it under a HillChunk, assign the
/// HumanNpc scene and (optionally) the NavigationRegion3D nodes to roam. If no
/// regions are assigned it discovers every NavigationRegion3D beneath the
/// parent chunk automatically.
/// </summary>
public partial class HumanNpcSpawner : Node3D
{
	[Export]
	public PackedScene HumanNpcScene;

	/// <summary>Total number of NPC instances kept alive in the pool.</summary>
	[Export]
	public int PoolSize = 8;

	/// <summary>Maximum number of NPCs roaming at the same time.</summary>
	[Export]
	public int MaxActive = 5;

	[Export]
	public float MinSpawnInterval = 2.0f;

	[Export]
	public float MaxSpawnInterval = 6.0f;

	[Export]
	public float MinSpeed = 1.0f;

	[Export]
	public float MaxSpeed = 2.5f;

	/// <summary>
	/// Navigation meshes the NPCs are allowed to roam. When left empty the
	/// spawner discovers every NavigationRegion3D under the owning chunk.
	/// </summary>
	[Export]
	public NavigationRegion3D[] NavigationRegions;

	private readonly List<NavigationRegion3D> _regions = new();
	private Stack<MobileNpc> _available;
	private HashSet<MobileNpc> _active;
	private RandomNumberGenerator _rng;
	private float _spawnTimer;

	public override void _Ready()
	{
		_rng = new RandomNumberGenerator();
		_rng.Randomize();

		_available = new Stack<MobileNpc>(PoolSize);
		_active = new HashSet<MobileNpc>();

		CollectRegions();

		if (HumanNpcScene == null)
		{
			GD.PrintErr("HumanNpcSpawner: HumanNpcScene is not assigned; no NPCs will spawn.");
			return;
		}

		for (int i = 0; i < PoolSize; i++)
		{
			var npc = HumanNpcScene.Instantiate<MobileNpc>();
			AddChild(npc);
			npc.AddToGroup("MovableObstacles");
			npc.OnReachedExit += HandleNpcReachedTarget;
			npc.Disable();
			_available.Push(npc);
		}

		_spawnTimer = _rng.RandfRange(MinSpawnInterval, MaxSpawnInterval);
	}

	public override void _Process(double delta)
	{
		SpawnTick(delta);
	}

	public void SpawnTick(double delta)
	{
		if (_available == null || _available.Count == 0)
			return;

		if (_active.Count >= MaxActive)
			return;

		_spawnTimer -= (float)delta;
		if (_spawnTimer > 0f)
			return;

		_spawnTimer = _rng.RandfRange(MinSpawnInterval, MaxSpawnInterval);

		// The navigation map may not have synced yet on the first few frames;
		// skip this tick and try again later if we can't find a valid point.
		if (!TryGetRandomNavPoint(out Vector3 spawnPoint))
			return;

		if (!TryGetRandomNavPoint(out Vector3 destination))
			destination = spawnPoint;

		var npc = _available.Pop();
		_active.Add(npc);

		float speed = _rng.RandfRange(MinSpeed, MaxSpeed);
		npc.Activate(spawnPoint, speed);
		npc.SetDestination(destination);
	}

	private void HandleNpcReachedTarget(object sender, EventArgs e)
	{
		if (sender is not MobileNpc npc)
			return;

		if (_active == null || !_active.Contains(npc))
			return;

		// Keep the pedestrian wandering: pick a fresh random destination.
		if (TryGetRandomNavPoint(out Vector3 destination))
			npc.SetDestination(destination);
	}

	/// <summary>Despawns a single roaming NPC back into the pool.</summary>
	public void ReturnNpc(MobileNpc npc)
	{
		if (npc == null || _active == null || !_active.Contains(npc))
			return;

		npc.Disable();
		_active.Remove(npc);
		_available.Push(npc);
	}

	/// <summary>Despawns every roaming NPC. Useful when a chunk is recycled.</summary>
	public void ReturnAll()
	{
		if (_active == null)
			return;

		var toReturn = new List<MobileNpc>(_active);
		foreach (var npc in toReturn)
			ReturnNpc(npc);
	}

	private bool TryGetRandomNavPoint(out Vector3 point)
	{
		point = Vector3.Zero;
		if (_regions.Count == 0)
			return false;

		for (int attempt = 0; attempt < _regions.Count; attempt++)
		{
			var region = _regions[_rng.RandiRange(0, _regions.Count - 1)];
			if (region == null)
				continue;

			Rid rid = region.GetRegionRid();
			if (!rid.IsValid)
				continue;

			Vector3 candidate = NavigationServer3D.RegionGetRandomPoint(rid, 1, false);
			// A zero vector signals the region/map wasn't ready or has no polygons.
			if (candidate != Vector3.Zero)
			{
				point = candidate;
				return true;
			}
		}

		return false;
	}

	private void CollectRegions()
	{
		_regions.Clear();

		if (NavigationRegions != null && NavigationRegions.Length > 0)
		{
			foreach (var region in NavigationRegions)
			{
				if (region != null)
					_regions.Add(region);
			}
			return;
		}

		// Fall back to discovering every region beneath the owning chunk.
		Node searchRoot = GetParent() ?? this;
		CollectRegionsRecursive(searchRoot);

		if (_regions.Count == 0)
			GD.PrintErr("HumanNpcSpawner: no NavigationRegion3D found to roam.");
	}

	private void CollectRegionsRecursive(Node node)
	{
		foreach (var child in node.GetChildren())
		{
			if (child is NavigationRegion3D region)
				_regions.Add(region);

			if (child.GetChildCount() > 0)
				CollectRegionsRecursive(child);
		}
	}
}
