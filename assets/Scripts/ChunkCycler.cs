using Godot;
using System;
using System.Collections.Generic;
using System.IO.Pipes;
using System.Linq;


public enum TravelDirection
{
	Inbound,
	Outbound
}

public static class GDExtensions

{

	public static T GetChildByType<T>(this Node node, bool recursive = true)
	where T : Node
	{
		int childCount = node.GetChildCount();
		for (int i = 0; i < childCount; i++)
		{
			Node child = node.GetChild(i);
			if (child is T childT)
			{
				return childT;
			}
			if (recursive && child.GetChildCount() > 0)
			{
				T recursiveResult = child.GetChildByType<T>(true);
				if (recursiveResult != null)
				{
					return recursiveResult;
				}
			}
		}
		return null;
	}

}
public partial class ChunkCycler : Node3D
{

	public float ChunkLength = 100;
	public float HillAngle = 17;
	public float WaterPlaneOffset = 30;
	public Node3D WaterPlane;
	public List<HillChunk> ChunkPool;
	private List<HillChunk> FrontPool;
	private Queue<HillChunk> PassedPool;
	[Export]
	public Node3D Player;
	[Export]
	public BoardController playerController;
	[Export]
	public TrafficPool TrafficPool;

	private TrafficSimulator _trafficSimulator;

	// Called when the node enters the scene tree for the first time.
	public override void _Ready()
	{
		ChunkPool = new List<HillChunk>();
		var sceneChunks = GetTree().GetNodesInGroup("Chunks");

		foreach (var c in sceneChunks)
		{
			if (c is HillChunk)
			{
				((HillChunk)c).ChunkPassed += HandleChunkPassed;
				ChunkPool.Add(c as HillChunk);
			}
		}
		FrontPool = new List<HillChunk>();
		PassedPool = new Queue<HillChunk>();
		FrontPool.AddRange(ChunkPool);
		playerController.PlayerRespawned += ChunkCycler_PlayerRespawned;

		_trafficSimulator = new TrafficSimulator();
		AddChild(_trafficSimulator);

		if (TrafficPool != null)
		{
			TrafficPool.SetSimulator(_trafficSimulator);
			foreach (var chunk in ChunkPool)
			{
				chunk.InitializeTraffic(TrafficPool, _trafficSimulator);
			}
		}
	}

	private void HandleChunkPassed(object sender, ChunkPassedEventArgs e)
	{
		MoveChunk(e.PassedChunk);
	}

	// Called every frame. 'delta' is the elapsed time since the previous frame.
	public override void _Process(double delta)
	{
		foreach (var chunk in ChunkPool)
		{
			if (chunk.Occupied && chunk.IsPositionResetChunk)
			{
				var pos = chunk.GlobalPosition;
				var distance = pos.DistanceTo(Vector3.Zero);
				foreach (var c in ChunkPool)
				{
					c.GlobalPosition -= pos;
				}
				Player.GlobalPosition -= pos;
				MoveObstacles(chunk, pos);
			}
		}
	}

	private void ChunkCycler_PlayerRespawned(object sender, PlayerRespawnArgs e)
	{
		TrafficPool?.ReturnAll();
		ResetChunks();
	}

	/// <summary>
	/// Put the chunks back in an order where the respawn chunk is at the top. Before if the respawn chunk was last there would be no transition chunk in front and they would
	/// start to pop up at the player on chunk exit
	/// </summary>
	public void ResetChunks()
	{
		HillChunk respawnChunk = null;
		List<HillChunk> contentChunks = new List<HillChunk>();
		foreach (var chunk in ChunkPool)
		{
			if (chunk.IsRespawnChunk)
			{
				respawnChunk = chunk;

				var pos = chunk.GlobalPosition;
				var distance = pos.DistanceTo(Vector3.Zero);
				chunk.GlobalPosition -= pos;
				chunk.CycleObstacles();
				chunk.Occupied = true;
			}
			else
			{
				contentChunks.Add(chunk);
			}
		}
		var chunkEndPos = respawnChunk.ChunkEnd.GlobalPosition;
		foreach (var contentChunk in contentChunks)
		{
			MoveChunkToPosition(contentChunk, chunkEndPos);
			chunkEndPos = contentChunk.ChunkEnd.GlobalPosition;
			contentChunk.Occupied = false;
			contentChunk.Passed = false;
		}
		this.PassedPool.Clear();

	}

	public bool TryGetNeighborChunk(HillChunk currentChunk, TravelDirection direction, out HillChunk neighborChunk)
	{
		var sortedChunks = ChunkPool.OrderBy(c => c.GlobalPosition.Z).ToArray();
		for (var i = 0; i < sortedChunks.Length; i++)
		{
			if (sortedChunks[i].Name == currentChunk.Name)
			{
				if (direction == TravelDirection.Inbound)
				{
					if (i + 1 <= sortedChunks.Length - 1)
					{
						neighborChunk = sortedChunks[i + 1];
						return true;
					}
				}
				else
				{
					if (i - 1 >= 0)
					{
						neighborChunk = sortedChunks[i - 1];
						return true;
					}
				}
			}
		}
		neighborChunk = null;
		return false;
	}

	void MoveChunk(HillChunk chunk)
	{
		PassedPool.Enqueue(chunk);
		if (PassedPool.Count > 1)
		{
			chunk = PassedPool.Dequeue();
		}
		else { return; }
		var minChunkEnd = FindMinChunkEnd();
		var dist = chunk.ChunkStart.GlobalPosition - minChunkEnd;
		chunk.GlobalPosition = (chunk.GlobalPosition - dist);
		GD.Print($"Moving {chunk.Name} from passed pool to {chunk.GlobalPosition}");
		chunk.Passed = false;
		MoveObstacles(chunk, dist);
		chunk.CycleObstacles();
	}

	void MoveChunkToPosition(HillChunk chunk, Vector3 position)
	{
		var dist = chunk.ChunkStart.GlobalPosition - position;
		chunk.GlobalPosition = (chunk.GlobalPosition - dist);
		chunk.Passed = false;
		MoveObstacles(chunk, dist);
		chunk.CycleObstacles();
	}

	Vector3 FindMinChunkEnd()
	{
		var min = Vector3.Zero;
		HillChunk minChunk = null;
		foreach (var c in ChunkPool)
		{
			if (c.ChunkEnd.GlobalPosition.Y < min.Y && c.ChunkEnd.GlobalPosition.Z > min.Z)
			{
				min = c.ChunkEnd.GlobalPosition;
				minChunk = c;
			}
		}
		return min;
	}

	private void MoveObstacles(HillChunk chunk, Vector3 dist)
	{
		var pos = chunk.GlobalPosition;
		var moveableObstacles = GetTree().GetNodesInGroup("MovableObstacles");
		foreach (var moveableObstacle in moveableObstacles)
		{
		}
	}

	IEnumerator<object> LowerWaterPlane()
	{
		var min = FindMinChunkEnd() - new Vector3(0, WaterPlaneOffset, 0);
		while (WaterPlane.GlobalPosition.Y > min.Y - WaterPlaneOffset)
		{
			WaterPlane.GlobalPosition.MoveToward(min, (float)this.GetPhysicsProcessDeltaTime() * 5f);
			yield return null;
		}
	}

}
