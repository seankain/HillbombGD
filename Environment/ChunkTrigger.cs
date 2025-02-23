using Godot;
using System;

public enum ChunkTriggerType
{
	Entry,
	Exit
}

public class ChunkTransitEventArgs { }

public delegate void ChunkEnteredEventHandler(object sender, ChunkTransitEventArgs e);
public delegate void ChunkExitedEventHandler(object sender, ChunkTransitEventArgs e);

public partial class ChunkTrigger : Area3D
{

	public event ChunkEnteredEventHandler ChunkEntered;
	public event ChunkExitedEventHandler ChunkExited;

	protected virtual void OnChunkEntered(ChunkTransitEventArgs e)
	{
		ChunkEnteredEventHandler handler = ChunkEntered;
		handler?.Invoke(this, e);
	}

	protected virtual void OnChunkExited(ChunkTransitEventArgs e)
	{
		ChunkExitedEventHandler handler = ChunkExited;
		handler?.Invoke(this, e);
	}

	public ChunkTriggerType TriggerType;
	// Start is called before the first frame update
	void Start()
	{

	}

	// Update is called once per frame
	void Update()
	{

	}
	//TODO These unity methods need to be swithched out
	// private void OnTriggerEnter(CollisionShape3D other)
	// {
	// 	if (other.CompareTag("Player") && TriggerType == ChunkTriggerType.Entry)
	// 	{
	// 		if (!other.GetComponent<BoardControllerBase>().Bailed)
	// 		{
	// 			OnChunkEntered(new ChunkTransitEventArgs());
	// 		}
	// 		//Debug.Log($"Player entering chunk");
	// 	}
	// }

	// private void OnTriggerExit(CollisionShape3D other)
	// {
	// 	if (other.CompareTag("Player") && TriggerType == ChunkTriggerType.Exit)
	// 	{
	// 		if (!other.GetComponent<BoardControllerBase>().Bailed)
	// 		{
	// 			OnChunkExited(new ChunkTransitEventArgs());
	// 		}
	// 		//Debug.Log($"Player exiting chunk");
	// 	}
	// }
	// Called when the node enters the scene tree for the first time.
	public override void _Ready()
	{
		this.BodyEntered += (o) =>
		{
			OnChunkEntered(new ChunkTransitEventArgs());
		};
		this.BodyExited += (o) =>
		{
			OnChunkExited(new ChunkTransitEventArgs());
		};
	}

	// Called every frame. 'delta' is the elapsed time since the previous frame.
	public override void _Process(double delta)
	{
	}
}
