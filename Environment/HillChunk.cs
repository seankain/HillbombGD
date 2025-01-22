using Godot;
using System;
public class ChunkPassedEventArgs { }

public delegate void ChunkPassedEventHandler(object sender, ChunkPassedEventArgs e);

public partial class HillChunk : Node3D
{

    public Node3D ChunkStart;
    public Node3D ChunkEnd;
    public bool IsPositionResetChunk;
    public bool Passed = false;
    public bool Occupied = false;
    public bool IsRespawnChunk = false;

    [Export]
    public ChunkTrigger EntryTrigger;
    [Export]
    public ChunkTrigger ExitTrigger;

    //public Waypoint InboundTopWaypoint;
    //public Waypoint InboundBottomWaypoint;
    //public Waypoint OutboundTopWaypoint;
    //public Waypoint OutboundBottomWaypoint;

    public event ChunkPassedEventHandler ChunkPassed;

    //[SerializeField]
    //private ParkedCar[] parkedCars;

    // Called when the node enters the scene tree for the first time.
    public override void _Ready()
    {
        EntryTrigger.ChunkEntered += HandleChunkEnter;
        ExitTrigger.ChunkExited += HandleChunkExit;
    }

    public void CycleObstacles()
    {
        // foreach (var p in parkedCars)
        // {
        //     p.Cycle();
        // }
    }

    protected virtual void OnChunkPassed(ChunkPassedEventArgs e)
    {
        ChunkPassedEventHandler handler = ChunkPassed;
        handler?.Invoke(this, e);
    }


    void HandleChunkEnter(object sender, ChunkTransitEventArgs e)
    {
        Occupied = true;
    }
    void HandleChunkExit(object sender, ChunkTransitEventArgs e)
    {
        Passed = true;
        Occupied = false;
        OnChunkPassed(new ChunkPassedEventArgs());

    }

    // // Called every frame. 'delta' is the elapsed time since the previous frame.
    // public override void _Process(double delta)
    // {
    // }
}
