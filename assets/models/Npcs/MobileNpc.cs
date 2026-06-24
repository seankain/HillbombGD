using Godot;
using System;
public enum NpcNavigationState
{
    Entering,
	Leaving,
	Evading
}

public partial class MobileNpc : CharacterBody3D, IObstacleType
{
	public event EventHandler OnReachedExit;
	[Export]
	public float Speed = 1.0f;
	public const float JumpVelocity = 4.5f;
	[Export]
	public NavigationAgent3D navigationAgent;
	[Export]
	public AnimationPlayer animationPlayer;

	[Export]
	public CollisionShape3D agentCollisionShape;

	[Export]
	public PhysicalBoneSimulator3D boneSimulator;

	[Export]
	public Node3D entrance;

	/// <summary>
	/// When true the NPC is driven by a <see cref="HumanNpcSpawner"/>: it does
	/// not seek the building entrance on its own and lets the spawner assign
	/// its position, speed and destinations via Activate/SetDestination.
	/// </summary>
	[Export]
	public bool ManagedBySpawner = false;

    public ObstacleType ObstacleType => ObstacleType.Human;


    public override void _Ready()
    {
        var root = GetTree().Root;
		navigationAgent.MaxSpeed = Speed;
		navigationAgent.TargetReached += HandleTargetReached;
		animationPlayer.Play("WalkPhone");

		if (ManagedBySpawner)
		{
			// The spawner controls lifecycle and destinations.
			return;
		}

		if(entrance == null)
        {
			entrance = root.GetNodeOrNull<Node3D>("/root/Main/Level/BuildingEntrance");
        }
		if (entrance != null)
		{
			navigationAgent.TargetPosition = entrance.GlobalPosition;
		}
    }

	/// <summary>Activates a pooled NPC at the given spot with the given speed.</summary>
	public void Activate(Vector3 spawnPosition, float speed)
	{
		Speed = speed;
		navigationAgent.MaxSpeed = speed;
		GlobalPosition = spawnPosition;
		Velocity = Vector3.Zero;
		Enable();
		animationPlayer.Play("WalkPhone");
	}

	/// <summary>Sets the NPC's navigation destination in world space.</summary>
	public void SetDestination(Vector3 worldPosition)
	{
		navigationAgent.TargetPosition = worldPosition;
	}

	public void Enable()
	{
		Visible = true;
		ProcessMode = ProcessModeEnum.Inherit;
		SetProcess(true);
		SetPhysicsProcess(true);
		if (agentCollisionShape != null)
			agentCollisionShape.Disabled = false;
	}

	public void Disable()
	{
		Velocity = Vector3.Zero;
		Visible = false;
		SetProcess(false);
		SetPhysicsProcess(false);
		ProcessMode = ProcessModeEnum.Disabled;
		if (agentCollisionShape != null)
			agentCollisionShape.Disabled = true;
	}

    private void HandleTargetReached()
    {
		OnReachedExit?.Invoke(this,new());
    }

    public override void _Process(double delta)
    {
        
    }

	public override void _PhysicsProcess(double delta)
	{
		Vector3 velocity = Velocity;

		// Add the gravity.
		if (!IsOnFloor())
		{
			velocity += GetGravity() * (float)delta;
		}
		var destination = navigationAgent.GetNextPathPosition();
		var localDestination = destination - this.GlobalPosition;
		var direction = localDestination.Normalized();
		velocity = direction * Speed;
		if(!this.GlobalPosition.IsEqualApprox(destination)){
		this.LookAt(destination);
		}
		// // Handle Jump.
		// if (Input.IsActionJustPressed("ui_accept") && IsOnFloor())
		// {
		// 	velocity.Y = JumpVelocity;
		// }

		// // Get the input direction and handle the movement/deceleration.
		// // As good practice, you should replace UI actions with custom gameplay actions.
		// Vector2 inputDir = Input.GetVector("ui_left", "ui_right", "ui_up", "ui_down");
		// Vector3 direction = (Transform.Basis * new Vector3(inputDir.X, 0, inputDir.Y)).Normalized();
		// if (direction != Vector3.Zero)
		// {
		// 	velocity.X = direction.X * Speed;
		// 	velocity.Z = direction.Z * Speed;
		// }
		// else
		// {
		// 	velocity.X = Mathf.MoveToward(Velocity.X, 0, Speed);
		// 	velocity.Z = Mathf.MoveToward(Velocity.Z, 0, Speed);
		// }

		 Velocity = velocity;
		MoveAndSlide();
		for(var i = 0; i < this.GetSlideCollisionCount(); i++)
        {
            var col = this.GetSlideCollision(i);
            if (((Node)col.GetCollider()).IsInGroup("Player"))
            {
                Ragdoll();
            }
        }

	}

	public void Ragdoll()
    {
		navigationAgent.SetProcess(false);
		this.SetProcess(false);
		this.SetPhysicsProcess(false);
		navigationAgent.SetPhysicsProcess(false);
        this.animationPlayer.Active = false;
		this.animationPlayer.SetProcess(false);
		boneSimulator.Active = true;
		boneSimulator.PhysicalBonesStartSimulation();
		agentCollisionShape.Disabled = true;

    }

	public void Unragdoll()
	{	
		boneSimulator.PhysicalBonesStopSimulation();
		boneSimulator.Active = false;
		agentCollisionShape.Disabled = false;
		this.SetProcess(true);
		this.SetPhysicsProcess(true);
		navigationAgent.SetPhysicsProcess(true);
        navigationAgent.SetProcess(true);
		this.animationPlayer.Active = true;
		this.animationPlayer.SetProcess(true);

	}
}
