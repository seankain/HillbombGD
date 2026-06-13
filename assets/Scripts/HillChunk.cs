using Godot;
using System;
public class ChunkPassedEventArgs { public HillChunk PassedChunk { get; set; } }

public delegate void ChunkPassedEventHandler(object sender, ChunkPassedEventArgs e);

public partial class HillChunk : Node3D
{
    [Export]
    public Node3D ChunkStart;
    [Export]
    public Node3D ChunkEnd;

    [Export]
    public Area3D EntryTriggerArea;
    [Export]
    public Area3D ExitTriggerArea;
    [Export]
    public bool IsPositionResetChunk;
    public bool Passed = false;
    public bool Occupied = false;
    [Export]
    public bool IsRespawnChunk = false;
    [Export]
    public TrafficSpawner Spawner;

    public ChunkTrigger EntryTrigger;
    public ChunkTrigger ExitTrigger;

    public event ChunkPassedEventHandler ChunkPassed;

    public override void _Ready()
    {
        EntryTrigger = EntryTriggerArea as ChunkTrigger;
        EntryTrigger.ChunkEntered += HandleChunkEnter;
        ExitTrigger = ExitTriggerArea as ChunkTrigger;
        ExitTrigger.ChunkExited += HandleChunkExit;
    }

    public void InitializeTraffic(TrafficPool pool, TrafficSimulator simulator = null)
    {
        if (Spawner != null && pool != null)
            Spawner.Initialize(pool, simulator);
    }

    public void CycleObstacles()
    {
        if (Spawner != null)
            Spawner.ReturnAllCars();
    }

    public override void _Process(double delta)
    {
        if (!Passed && Spawner != null)
            Spawner.SpawnTick(delta);
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
        OnChunkPassed(new ChunkPassedEventArgs { PassedChunk = this });
    }
}
