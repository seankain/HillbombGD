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
public partial class ChunkCycler : Node3D
{

	public float ChunkLength = 100;
	public float HillAngle = 17;
	public float WaterPlaneOffset = 30;
	public Node3D WaterPlane;
	public List<HillChunk> ChunkPool;
	private List<HillChunk> FrontPool;
	private List<HillChunk> PassedPool;
	public Node3D Player;
	private roller playerController;

	// Start is called before the first frame update
	void Start()
	{
		FrontPool = new List<HillChunk>();
		PassedPool = new List<HillChunk>();
		FrontPool.AddRange(ChunkPool);
		//TODO rewire player respawn event
		// playerController = Player.GetComponent<BoardControllerBase>();
		// playerController.PlayerRespawned += ChunkCycler_PlayerRespawned;
	}

	private void ChunkCycler_PlayerRespawned(object sender, System.EventArgs e)
	{
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

	}

	// Update is called once per frame
	void Update()
	{
		foreach (var chunk in ChunkPool)
		{
			if (chunk.Occupied && chunk.IsPositionResetChunk)
			{
				//Going to move all of the chunks back to around zero to avoid floating point issues from sustained play
				//var pos = chunk.gameObject.transform.position;
				var pos = chunk.GlobalPosition;
				var distance = pos.DistanceTo(Vector3.Zero);
				foreach (var c in ChunkPool)
				{
					c.GlobalPosition -= pos;
				}
				Player.GlobalPosition -= pos;
				MoveObstacles(chunk, pos);
			}
			if (chunk.Passed)
			{
				MoveChunk(chunk);
			}
		}
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
					//Go forward in z axis since chunks progress in z axis and inbound is positive on z
					if (i + 1 <= sortedChunks.Length - 1)
					{
						neighborChunk = sortedChunks[i + 1];
						return true;
					}
				}
				else
				{
					//Go backward in z axis
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
		var minChunkEnd = FindMinChunkEnd();
		var dist = chunk.ChunkStart.GlobalPosition - minChunkEnd;
		chunk.GlobalPosition = (chunk.GlobalPosition - dist);
		//Debug.Log($"Moving {chunk.name} to  {chunk.transform.position}");
		chunk.Passed = false;
		//WaterPlane.transform.position = new Vector3(WaterPlane.transform.position.x, minChunkEnd.y - WaterPlaneOffset, WaterPlane.transform.position.z);
		MoveObstacles(chunk, dist);
		//TODO fix water plane
		//StartCoroutine(LowerWaterPlane());
		chunk.CycleObstacles();
	}

	void MoveChunkToPosition(HillChunk chunk, Vector3 position)
	{
		var dist = chunk.ChunkStart.GlobalPosition - position;
		chunk.GlobalPosition = (chunk.GlobalPosition - dist);
		//Debug.Log($"Moving {chunk.name} to  {chunk.transform.position}");
		chunk.Passed = false;
		//WaterPlane.transform.position = new Vector3(WaterPlane.transform.position.x, minChunkEnd.y - WaterPlaneOffset, WaterPlane.transform.position.z);
		MoveObstacles(chunk, dist);
		Coroutines.StartCoroutine((System.Collections.IEnumerable)LowerWaterPlane());
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
		//Debug.Log($"Min chunk is {minChunk.gameObject.name} at {min}");
		return min;
	}

	private void MoveObstacles(HillChunk chunk, Vector3 dist)
	{
		var pos = chunk.GlobalPosition;
		//TODO this group isnt being populated yet so when there arent
		//any to be found this is why
		var moveableObstacles = GetTree().GetNodesInGroup("MovableObstacles");
		//var npcVehicles = FindObjectsOfType<NpcVehicle>();
		foreach (var moveableObstacle in moveableObstacles)
		{
			//TODO havent moved over any obstacle scripts yet 
			// var npcVehicle = moveableObstacle.GetChild<NpcVehicle>();
			// if (npcVehicle.CurrentChunk == chunk)
			// {

			// 	npcVehicle.transform.position = (npcVehicle.transform.position - dist);
			// }
		}
	}

	IEnumerator<object> LowerWaterPlane()
	{
		var min = FindMinChunkEnd() - new Vector3(0, WaterPlaneOffset, 0);
		while (WaterPlane.GlobalPosition.Y > min.Y - WaterPlaneOffset)
		{
			WaterPlane.GlobalPosition.MoveToward(min, (float)this.GetPhysicsProcessDeltaTime() * 5f);
			//WaterPlane.GlobalPosition = Vector3.MoveTowards(WaterPlane.GlobalPosition, min, Time.deltaTime * 5f);
			yield return null;
		}
	}

	// Called when the node enters the scene tree for the first time.
	public override void _Ready()
	{
	}

	// Called every frame. 'delta' is the elapsed time since the previous frame.
	public override void _Process(double delta)
	{
	}

}