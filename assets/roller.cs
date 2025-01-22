using Godot;
using System;

public partial class roller : RigidBody3D
{

	[Export]
	public Marker3D CameraRig;
	[Export]
	public RayCast3D FloorCheck;
	[Export]
	public float RollingForce = 40f;
	[Export]
	public float JumpForce = 1000f;


	// Called when the node enters the scene tree for the first time.
	public override void _Ready()
	{
		CameraRig.TopLevel = true;
		FloorCheck.TopLevel = true;

	}

	// Called every frame. 'delta' is the elapsed time since the previous frame.
	public override void _Process(double delta)
	{
	}

	public override void _PhysicsProcess(double delta)
	{
		//TODO downcast okay in this case?
		//var ucrt = CameraRig.GlobalTransform;
		//ucrt.InterpolateWith(GlobalTransform, (float)delta);
		CameraRig.GlobalTransform.InterpolateWith(GlobalTransform, (float)delta);
		//CameraRig.GlobalTransform.Origin = Godot.Lerp(CameraRig.GlobalTransform.Origin,GlobalTransform.Origin,0.1);
		//base._PhysicsProcess(delta);

		//$FloorCheck.global_transform.origin = global_transform.origin
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
			av.X -= RollingForce * (float)delta;
		}
		else if (Input.IsActionPressed("Back"))
		{
			av.X += RollingForce * (float)delta;
		}
		else if (Input.IsActionPressed("Left"))
		{
			av.Z += RollingForce * (float)delta;
		}
		else if (Input.IsActionPressed("Right"))
		{
			av.Z -= RollingForce * (float)delta;
		}
		this.AngularVelocity = av;

		if (Input.IsActionJustPressed("Jump") && FloorCheck.IsColliding())
		{
			ApplyImpulse(Vector3.Zero, Vector3.Up * JumpForce);
		}
		/*
		 # As the ball moves, move the raycast along with it
    
    if Input.is_action_pressed("forward"):
        angular_velocity.x -= rolling_force*delta
    elif Input.is_action_pressed("back"):
        angular_velocity.x += rolling_force*delta
    if Input.is_action_pressed("left"):
        angular_velocity.z += rolling_force*delta
    elif Input.is_action_pressed("right"):
        angular_velocity.z -= rolling_force*delta

    # When the ball is on the floor and the user presses jump button,
    # add impulse moving the ball up.
    if Input.is_action_just_pressed("jump") and $FloorCheck.is_colliding():
        apply_impulse(Vector3(), Vector3.UP*1000)

		*/
	}
}
