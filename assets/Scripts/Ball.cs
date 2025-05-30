using Godot;
using System;


public class PlayerRespawnArgs { }

public delegate void PlayerRespawnedEventHandler(object sender, PlayerRespawnArgs e);

public partial class Ball : RigidBody3D
{

	[Export]
	public Marker3D CameraRig;
	[Export]
	public Marker3D CameraXPivot;

	[Export]
	public float CameraChaseSpeed = 1.0f;

	[Export]
	public RayCast3D FloorCheck;
	[Export]
	public float RollingForce = 40f;
	[Export]
	public float JumpForce = 1000f;
	[Export]
	public float MouseSensitivity = 0.1f;

	[Export]
	public float DeathForce = 100f;

	[Export]
	public PackedScene GibScene;

	private Gibsplosion gibber;

	public PlayerRespawnedEventHandler PlayerRespawned;

	private bool respawnPressed = false;

	private bool isDead = false;

	private Vector3 prevVelocity = Vector3.Zero;


	// Called when the node enters the scene tree for the first time.
	public override void _Ready()
	{
		CameraRig.TopLevel = true;
		FloorCheck.TopLevel = true;
		BodyEntered += HandleCollision;
		Input.MouseMode = Input.MouseModeEnum.Captured;
		var gibScene = GibScene.Instantiate();
		//GetTree().Root.AddChild(gibScene);
		GetNode("../HiddenPlace").AddChild(gibScene);
		gibber = gibScene as Gibsplosion;
	}

	private void HandleCollision(Node body)
	{
		// Get the acceleration (change in velocity per second)
		if (isDead) { return; }
		Vector3 acceleration = (LinearVelocity - prevVelocity) / (float)GetPhysicsProcessDeltaTime();
		var force = Mass * acceleration;
		//GD.Print(force);
		if (force.Length() >= DeathForce)
		{
			GD.Print(force.Length());
			//GD.Print("You died.");
			Die();
		}

	}

	public override void _Input(InputEvent @event)
	{
		if (@event is InputEventMouseMotion)
		{
			var mouseEvent = (InputEventMouseMotion)@event;
			CameraRig.RotateY(-Mathf.DegToRad(mouseEvent.Relative.X * this.MouseSensitivity));
			CameraXPivot.RotateX(-Mathf.DegToRad(mouseEvent.Relative.Y * this.MouseSensitivity));


			//CameraRig.RotateCameraRig.CameraRig.GlobalPosition.Slerp(GlobalPosition, 0.1f);
			//CameraRig.GlobalPosition = RotateAround(CameraRig.GlobalTransform, GlobalPosition, 2f, 2f);
		}
	}

	private Vector3 RotateAround(Transform3D transform, Vector3 center, float radius, float speed)
	{
		return center + new Vector3(
			Mathf.Sin(speed) * radius,
			0,
			Mathf.Cos(speed) * radius
		);


	}

	public override void _UnhandledInput(InputEvent @event)
	{
		if (@event is InputEventKey eventKey)
		{
			if (eventKey.Pressed && eventKey.Keycode == Key.Escape)
			{
				GetTree().Quit();
			}
			if (eventKey.Pressed && eventKey.Keycode == Key.R)
			{
				//Respawn();
				respawnPressed = true;
			}
		}
	}

	// Called every frame. 'delta' is the elapsed time since the previous frame.
	public override void _Process(double delta)
	{
	}

	private void UpdateCameraRigPosition(double delta)
	{
		//CameraRig.GlobalPosition = 
		var updatedPos = CameraRig.GlobalPosition.Lerp(GlobalPosition, (float)delta * CameraChaseSpeed);
		// TODO Get mouse position info 
		//updatedPos = RotateAround(CameraRig.GlobalTransform, updatedPos, 2f, 2f);
		//CameraRig.LookAtFromPosition(updatedPos, GlobalPosition);
		CameraRig.GlobalPosition = updatedPos;
	}

	public override void _PhysicsProcess(double delta)
	{
		//CameraRig.GlobalPosition = CameraRig.GlobalPosition.Lerp(GlobalPosition, (float)delta);
		UpdateCameraRigPosition(delta);
		if (isDead) { return; }
		var upfct = FloorCheck.GlobalTransform;
		upfct.Origin = new Vector3(
			this.Transform.Origin.X,
			this.Transform.Origin.Y,
			this.Transform.Origin.Z
		);
		FloorCheck.GlobalTransform = upfct;
		var av = this.AngularVelocity;
		if (Input.IsActionPressed("Forward"))
		{
			av.X += RollingForce * (float)delta;
		}
		else if (Input.IsActionPressed("Backward"))
		{
			av.X -= RollingForce * (float)delta;
		}
		else if (Input.IsActionPressed("Left"))
		{
			av.Z -= RollingForce * (float)delta;
		}
		else if (Input.IsActionPressed("Right"))
		{
			av.Z += RollingForce * (float)delta;
		}
		this.AngularVelocity = av;

		if (Input.IsActionJustPressed("Jump") && FloorCheck.IsColliding())
		{
			ApplyImpulse(Vector3.Zero, Vector3.Up * JumpForce);
		}
		prevVelocity = LinearVelocity;
	}

	public override void _IntegrateForces(PhysicsDirectBodyState3D state)
	{
		base._IntegrateForces(state);
		if (respawnPressed)
		{
			Respawn();
			respawnPressed = false;
		}

	}

	private void Die()
	{
		//var gibScene = GibScene.Instantiate();
		//AddChild(gibScene);
		isDead = true;
		this.GetChildByType<MeshInstance3D>().Visible = false;
		this.SetPhysicsProcess(false);
		gibber.Enable();
		gibber.Teleport(this.GlobalPosition);
		gibber.Gibsplode(1, LinearVelocity, AngularVelocity);
	}

	private void Respawn()
	{
		isDead = false;
		var respawn = GetTree().GetNodesInGroup("Respawn")[0] as Node3D;
		this.GlobalTransform = new Transform3D(respawn.GlobalTransform.Basis, respawn.GlobalPosition);
		this.PlayerRespawned?.Invoke(this, new PlayerRespawnArgs());
		this.LinearVelocity = Vector3.Zero;
		this.AngularVelocity = Vector3.Zero;
		this.GetChildByType<MeshInstance3D>().Visible = true;
		this.SetPhysicsProcess(true);
	}
}
