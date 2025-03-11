using Godot;
using System;

public partial class BoardController : CharacterBody3D
{
	public const float Speed = 5.0f;
	public const float JumpVelocity = 4.5f;

	[Export]
	public double AirDensity = 1.225;
	// average human drag coefficient
	[Export]
	public double DragCoefficient = 1.0;
	//for drag, meters squared
	[Export]
	public double ReferenceArea = 0.015;
	[Export]
	public double FrictionCoefficient = 0.02;

	public Node3D LeftFrontWheel;
	public Node3D RightFrontWheel;
	public Node3D LeftRearWheel;
	public Node3D RightRearWheel;
	public Node3D FrontAxle;
	public Node3D RearAxle;
	public bool SnapToSurface = true;
	public float UprightStability = 0.3f;
	public float UprightSpeed = 2.0f;
	public bool UprightSingleAxis = false;
	private WheelInfo[] wheelInfos;
	public Vector3 OlliePopForce = new Vector3(0, 1000, 0);
	public float SurfaceMatchSmoothing = 1f;
	public float SmoothDampVelocity = 5f;
	public float PlayerRotateSpeed = 5f;
	public float AirRotateSpeed = 15f;
	public float MaxTurnAngle = 5f;
	public float TurnSpeed = 1f;
	public float RayGroundDistance = 1f;
	public float CastDistance = 0.1f;
	public float SuspensionRestDistance = 0.1f;
	public float Damping = 30f;
	public float Strength = 100f;
	public float Offset = 0.1f;
	public float WheelGripFactor = 0.1f;
	public float WheelMass = 1;
	//private RigidBody3D rb;
	private float horizontal = 0f;
	private float vertical = 0f;
	private float currentTurnAngle = 0f;
	[Export]
	public float RollingResistanceCoefficient = 0.5f;
	private bool Grounded = false;
	//TODO consider making separate monobehavior theres too much in here
	private bool isGrinding = false;
	//private Grindable currentGrindingSurface = null;
	//private Vector3 GrindVelocity = Vector3.zero;
	private bool isJumping;

	public PlayerRespawnedEventHandler PlayerRespawned;

	// [SerializeField]
	// private CharacterState characterState;
	// [SerializeField]
	// private Animator SkateboardAnimator;

	void Start()
	{
		//rb = GetComponent<Rigidbody>();

		wheelInfos = new WheelInfo[4]
		{
			new WheelInfo(LeftFrontWheel),
			new WheelInfo(RightFrontWheel),
			new WheelInfo(LeftRearWheel),
			new WheelInfo(RightRearWheel)
		};
		// Turning the colliders on messed with the inertia tensors and center of mass so manually setting them
		// until i can figure out a way to do it correctly
		//rb.inertiaTensor = new Vector3(11.78971f, 12.80615f, 1.080781f);
		//rb.inertiaTensorRotation = Quaternion.Euler(0, 0, 359.9538f);
		//rb.centerOfMass = new Vector3(0, -0.0900267f, 0.03401519f);
		//GetComponent<BoxCollider>().enabled = true;
		//GetComponent<CapsuleCollider>().enabled = true;
		//rb.ResetInertiaTensor();
	}

	// Update is called once per frame
	// void Update(float deltaTime)
	// {
	// 	if (Input.IsActionPressed("Reset"))
	// 	{
	// 		Respawn();
	// 	}
	// 	Vector2 directional = Input.GetVector("Left", "Right", "Forward", "Backward");
	// 	isJumping = Input.IsActionPressed("Jump");
	// 	if (isGrinding)
	// 	{
	// 		//forces handled in fixed update
	// 		//handle grinding inputs separately from normal riding
	// 		//if (currentGrindingSurface.TryGetNextWaypoint(transform.position, out Transform nextGrindPosition))
	// 		//{
	// 		//    rb.velocity = Vector3.MoveTowards(transform.position, nextGrindPosition.position, GrindVelocity.z * Time.deltaTime);
	// 		//}
	// 		//if (isJumping)
	// 		//{
	// 		//    isGrinding = false;
	// 		//    currentGrindingSurface = null;
	// 		//    Ollie();
	// 		//}
	// 		//else
	// 		//{
	// 		//    return;
	// 		//}
	// 	}
	// 	if (isJumping && Grounded)
	// 	{
	// 		Ollie();
	// 		return;
	// 	}
	// 	currentTurnAngle += horizontal * TurnSpeed * deltaTime;
	// 	if (horizontal == 0)
	// 	{
	// 		currentTurnAngle -= TurnSpeed * deltaTime;
	// 		if (!Mathf.IsZeroApprox(currentTurnAngle))
	// 		{
	// 			currentTurnAngle = 0;
	// 		}
	// 	}
	// 	currentTurnAngle = Mathf.Clamp(currentTurnAngle, -MaxTurnAngle, MaxTurnAngle);
	// 	//Debug.Log(Vector3.Dot(transform.forward, Vector3.up));
	// 	if (Transform.Basis.Z.Dot(Vector3.Up) <= 0)
	// 	{
	// 		FrontAxle.RotationDegrees = new Vector3(0, currentTurnAngle, 0);

	// 		RearAxle.RotationDegrees = new Vector3(0, -currentTurnAngle, 0);
	// 	}
	// 	else
	// 	{
	// 		FrontAxle.RotationDegrees = new Vector3(0, -currentTurnAngle, 0);
	// 		RearAxle.RotationDegrees = new Vector3(0, currentTurnAngle, 0);
	// 	}

	// 	var ray = new Ray(transform.position, Vector3.down);
	// 	//Debug.DrawRay(transform.position, Vector3.down, Color.yellow);

	// 	if (SnapToSurface)
	// 	{
	// 		foreach (var wheelInfo in wheelInfos)
	// 		{
	// 			if (wheelInfo.Hitting)
	// 			{
	// 				transform.rotation = Quaternion.FromToRotation(wheelInfo.WheelTransform.up, wheelInfo.SurfaceNormal) * transform.rotation;
	// 			}
	// 		}
	// 	}

	// 	if (Physics.Raycast(ray, out var hitInfo, 1, ~(1 << LayerMask.NameToLayer("Skateboard"))))
	// 	{
	// 		if (hitInfo.distance < RayGroundDistance)
	// 		{
	// 			Grounded = true;
	// 			if (SnapToSurface)
	// 			{
	// 				//    //TODO smooth these out, also the weird snapping that happens when the character gets a raycast hit but isnt even close to rotated correctly
	// 				//    //gameObject.transform.rotation = SmoothDampQuaternion(gameObject.transform.rotation, Quaternion.Euler(hitInfo.normal),ref SmoothDampVelocity,SurfaceMatchSmoothing );
	// 				//    //gameObject.transform.rotation = Quaternion.Slerp(gameObject.transform.rotation, Quaternion.Euler(hitInfo.normal),SmoothDampVelocity);
	// 				//    //-------------------
	// 				//    //var n = new Vector3(hitInfo.normal.x, transform.rotation.eulerAngles.y, hitInfo.normal.z);
	// 				//    //gameObject.transform.rotation *= Quaternion.FromToRotation(gameObject.transform.up, n);
	// 				//    //gameObject.transform.Rotate(Vector3.down, -horizontal * PlayerRotateSpeed * Time.deltaTime);
	// 				//    //This is finnicky still and has some problems compared to the single raycast from the middle
	// 				//    foreach (var wheelInfo in wheelInfos)
	// 				//    {
	// 				//        if (wheelInfo.Hitting)
	// 				//        {
	// 				//            transform.rotation = Quaternion.FromToRotation(wheelInfo.WheelTransform.up, wheelInfo.SurfaceNormal) * transform.rotation;
	// 				//        }
	// 				//        //transform.rotation = Quaternion.LookRotation(wheelInfo.HitWorldLocation - wheelInfo.WheelTransform.position) * _childRPos;
	// 				//    }

	// 				//This works for the single central raycast but its too jerky and unresponsive to small terrain changes or slopes
	// 				//uncomment the aligning with surface normal to use
	// 				float headingDeltaAngle = Input.GetAxis("Horizontal") * Time.deltaTime * TurnSpeed;
	// 				Quaternion headingDelta = Quaternion.AngleAxis(headingDeltaAngle, transform.up);
	// 				//align with surface normal
	// 				//transform.rotation = Quaternion.FromToRotation(transform.up, hitInfo.normal) * transform.rotation;
	// 				//apply heading rotation
	// 				transform.rotation = headingDelta * transform.rotation;
	// 			}
	// 		}

	// 	}
	// 	else
	// 	{
	// 		Grounded = false;
	// 		//rb.AddTorque(-transform.up * (-horizontal * AirRotateSpeed * Time.deltaTime));
	// 		//gameObject.transform.Rotate(Vector3.down, -horizontal * AirRotateSpeed * Time.deltaTime);
	// 		//gameObject.transform.Rotate(Vector3.right, vertical * AirRotateSpeed * Time.deltaTime);
	// 		//rb.AddTorque(transform.right * (vertical * AirRotateSpeed * Time.deltaTime));
	// 	}


	// }


	// private static Quaternion SmoothSlerp(Transform current, Vector3 goalPosition, float speed)
	// {
	// 	var direction = (goalPosition - current.position).normalized;
	// 	var goal = Quaternion.LookRotation(direction);
	// 	return Quaternion.Slerp(current.rotation, goal, speed);
	// }

	// private static Quaternion SmoothDampQuaternion(Quaternion current, Quaternion target, ref float velocity, float smoothTime)
	// {
	// 	Vector3 c = current.eulerAngles;
	// 	Vector3 t = target.eulerAngles;
	// 	return Quaternion.Euler(
	// 	  Mathf.SmoothDampAngle(c.x, t.x, ref velocity, smoothTime),
	// 	  Mathf.SmoothDampAngle(c.y, t.y, ref velocity, smoothTime),
	// 	  Mathf.SmoothDampAngle(c.z, t.z, ref velocity, smoothTime)
	// 	);
	// }


	// private void AdjustWheelSuspension(Transform wheel, Vector3 wheelWorldVelocity)
	// {
	// 	var ray = new Ray(wheel.position, Vector3.down);
	// 	Debug.DrawRay(wheel.position, Vector3.down, Color.red);
	// 	if (Physics.Raycast(ray, out var hitInfo, CastDistance, ~(1 << LayerMask.NameToLayer("Skateboard"))))
	// 	{
	// 		//Credit to Toyful games from their Very Very Valet tutorial
	// 		// world-space velocity of wheel
	// 		Vector3 springDir = wheel.up;
	// 		//calculate the offset from the raycast
	// 		float offset = SuspensionRestDistance - hitInfo.distance;
	// 		//calculate velocity along the spring direction
	// 		// note that springDir is a unit vector, so this returns the magnitude of wheel world velocity
	// 		// as projected onto spring dir
	// 		float vel = Vector3.Dot(springDir, wheelWorldVelocity);
	// 		//calculate the magnitude of the damped spring force
	// 		float force = (offset * Strength) - (vel * Damping);
	// 		rb.AddForceAtPosition(springDir * force, wheel.position);
	// 	}
	// }

	// private void AdjustWheelSlip(Transform wheel)
	// {
	// 	Vector3 steeringDir = wheel.right;
	// 	//worldspace velocity of the suspension
	// 	Vector3 wheelWorldVel = rb.GetPointVelocity(wheel.position);
	// 	//what is the wheels velocity in the steering direction?
	// 	// note that steeringDir is a unit vector, so this returns the magnitude of wheelWorldVel
	// 	float steeringVel = Vector3.Dot(steeringDir, wheelWorldVel);
	// 	// the change in velocity that we're looking for is -steeringVel*gripFactor
	// 	// grip factor is in range 0-1,0 means no grip, 1 means full grip
	// 	float desiredVelChange = -steeringVel * WheelGripFactor;
	// 	//turn change in velocity into an acceleration
	// 	//this will produce the acceleration necessary to change the velocity by desiredVelChange in 1 physics step
	// 	float desiredAccel = desiredVelChange / Time.fixedDeltaTime;
	// 	rb.AddForceAtPosition(steeringDir * WheelMass * desiredAccel, wheel.position);
	// }

	// private void AddSteeringForces(WheelInfo wheelInfo)
	// {
	// 	var wheel = wheelInfo.WheelTransform;
	// 	//Credit to Toyful games from their Very Very Valet tutorial
	// 	var ray = new Ray(wheel.position, Vector3.down * CastDistance);
	// 	Debug.DrawRay(wheel.position, Vector3.down * CastDistance, Color.red);
	// 	if (Physics.Raycast(ray, out var hitInfo, CastDistance, ~(1 << LayerMask.NameToLayer("Skateboard"))))
	// 	{
	// 		wheelInfo.SurfaceNormal = hitInfo.normal;
	// 		wheelInfo.Hitting = true;
	// 		wheelInfo.HitWorldLocation = hitInfo.point;
	// 		//Spring
	// 		Vector3 wheelWorldVel = rb.GetPointVelocity(wheel.position);
	// 		//Credit to Toyful games from their Very Very Valet tutorial
	// 		// world-space velocity of wheel
	// 		Vector3 springDir = wheel.up;
	// 		//calculate the offset from the raycast
	// 		float offset = SuspensionRestDistance - hitInfo.distance;
	// 		//calculate velocity along the spring direction
	// 		// note that springDir is a unit vector, so this returns the magnitude of wheel world velocity
	// 		// as projected onto spring dir
	// 		float vel = Vector3.Dot(springDir, wheelWorldVel);
	// 		//calculate the magnitude of the damped spring force
	// 		float force = (offset * Strength) - (vel * Damping);
	// 		rb.AddForceAtPosition(springDir * force, wheel.position);

	// 		//Steering
	// 		Vector3 steeringDir = wheel.right;
	// 		//worldspace velocity of the suspension
	// 		//what is the wheels velocity in the steering direction?
	// 		// note that steeringDir is a unit vector, so this returns the magnitude of wheelWorldVel
	// 		float steeringVel = Vector3.Dot(steeringDir, wheelWorldVel);
	// 		// the change in velocity that we're looking for is -steeringVel*gripFactor
	// 		// grip factor is in range 0-1,0 means no grip, 1 means full grip
	// 		float desiredVelChange = -steeringVel * WheelGripFactor;
	// 		//turn change in velocity into an acceleration
	// 		//this will produce the acceleration necessary to change the velocity by desiredVelChange in 1 physics step
	// 		float desiredAccel = desiredVelChange / Time.fixedDeltaTime;
	// 		rb.AddForceAtPosition(steeringDir * WheelMass * desiredAccel, wheel.position);
	// 	}
	// 	else
	// 	{
	// 		wheelInfo.Hitting = false;
	// 	}
	// }

	// private void OnCollisionEnter(Collision collision)
	// {
	// 	if (collision.gameObject.tag == "Grindable")
	// 	{
	// 		GD.Print("Grinding");
	// 		GrindVelocity = new Vector3(0, 0, rb.velocity.z);
	// 		currentGrindingSurface = collision.gameObject.GetComponent<Grindable>();
	// 		transform.position = currentGrindingSurface.GetInitPosition(collision.contacts[0].point);
	// 		//rb.velocity = new Vector3(0, 0, 0);
	// 		isGrinding = true;
	// 	}
	// }

	// private void OnCollisionStay(Collision collision)
	// {
	// 	if (collision.gameObject.tag == "Grindable")
	// 	{
	// 		Debug.Log(gameObject.name);

	// 	}
	// }

	// private void OnCollisionExit(Collision collision)
	// {
	// 	if (collision.gameObject.tag == "Grindable")
	// 	{
	// 		GD.Print("Stop grinding");
	// 		isGrinding = false;
	// 		currentGrindingSurface = null;
	// 	}
	// }

	// private void Ollie()
	// {
	// 	characterState.Ollie();
	// 	SkateboardAnimator.SetTrigger("Ollie");
	// 	rb.AddForce(OlliePopForce);
	// }

	private void Respawn()
	{
		var respawn = GetTree().GetNodesInGroup("Respawn")[0] as Node3D;
		this.GlobalTransform = new Transform3D(respawn.GlobalTransform.Basis, respawn.GlobalPosition);
		this.PlayerRespawned?.Invoke(this, new PlayerRespawnArgs());
		this.Velocity = Vector3.Zero;
	}

	// private void FixedUpdate()
	// {
	// 	if (isGrinding)
	// 	{
	// 		if (currentGrindingSurface.TryGetNextWaypoint(transform.position, out Transform nextGrindPosition))
	// 		{
	// 			rb.AddForce(Vector3.MoveTowards(transform.position, nextGrindPosition.position, GrindVelocity.z * Time.fixedDeltaTime));
	// 		}
	// 		if (isJumping)
	// 		{
	// 			isGrinding = false;
	// 			currentGrindingSurface = null;
	// 			Ollie();
	// 		}
	// 		else
	// 		{
	// 			return;
	// 		}
	// 		return;
	// 	}
	// 	foreach (var wheelInfo in wheelInfos)
	// 	{
	// 		AddSteeringForces(wheelInfo);
	// 	}
	// 	if (!Grounded)
	// 	{
	// 		rb.AddTorque(-transform.up * (-horizontal * AirRotateSpeed * Time.fixedDeltaTime));
	// 		rb.AddTorque(transform.right * (vertical * AirRotateSpeed * Time.fixedDeltaTime));
	// 	}
	// 	//trying to find a way to stop wacky flipping without locking rigidbody
	// 	//https://answers.unity.com/questions/10425/how-to-stabilize-angular-motion-alignment-of-hover.html
	// 	var predictedUp = Quaternion.AngleAxis(rb.angularVelocity.magnitude * Mathf.Rad2Deg * UprightStability / UprightSpeed, rb.angularVelocity) * transform.up;
	// 	var torqueVector = Vector3.Cross(predictedUp, Vector3.up);
	// 	if (UprightSingleAxis)
	// 	{
	// 		Vector3.Project(torqueVector, transform.forward);
	// 	}
	// 	rb.AddTorque(torqueVector * UprightSpeed * UprightSpeed);
	// 	//foreach (var wheel in wheels)
	// 	//{
	// 	//    AddSteeringForces(wheel);
	// 	//}
	// }


	// Get the gravity from the project settings to be synced with RigidBody nodes.
	public float gravity = ProjectSettings.GetSetting("physics/3d/default_gravity").AsSingle();

	public override void _Input(InputEvent @event)
	{
		if (@event.IsActionPressed("Reset"))
		{
			Respawn();
		}
	}

	private double CalculateVelocity(double deltaTime, double hillAngle, double currentVelocity)
	{
		const double gravity = 9.81; // acceleration due to gravity (m/s^2)
									 // standard air density kg per meter cubed

		// Convert the hill angle to radians
		double hillAngleRadians = hillAngle * Math.PI / 180.0;

		// Calculate acceleration due to gravity component along the slope
		double gravityComponent = gravity * Math.Sin(hillAngleRadians);

		// Calculate rolling resistance force (simplified model)
		double rollingResistance = RollingResistanceCoefficient * gravity * Math.Cos(hillAngleRadians);

		double dragForce = 0.5 * DragCoefficient * AirDensity * Math.Pow(currentVelocity, 2) * ReferenceArea;

		// Calculate friction force
		double frictionForce = FrictionCoefficient * gravity * Math.Cos(hillAngleRadians);

		// Calculate net acceleration
		double netAcceleration = gravityComponent - rollingResistance - dragForce - frictionForce;

		// Update current velocity using the kinematic equation: v = u + at
		currentVelocity += netAcceleration * deltaTime;

		return currentVelocity;
	}

	public override void _PhysicsProcess(double delta)
	{
		Vector3 velocity = Velocity;

		// Add the gravity.
		if (!IsOnFloor())
			velocity.Y -= gravity * (float)delta;

		// Handle Jump.
		if (Input.IsActionJustPressed("Jump") && IsOnFloor())
		{
			velocity.Y = JumpVelocity;
		}

		// Get the input direction and handle the movement/deceleration.
		// As good practice, you should replace UI actions with custom gameplay actions.
		Vector2 inputDir = Input.GetVector("Left", "Right", "Forward", "Backward");
		Vector3 direction = (Transform.Basis * new Vector3(inputDir.X, 0, inputDir.Y)).Normalized();
		if (direction != Vector3.Zero)
		{
			velocity.X = -direction.X * Speed;
			//velocity.Z = direction.Z * Speed;
		}
		else
		{
			velocity.X = Mathf.MoveToward(Velocity.X, 0, Speed);
			//velocity.Z = Mathf.MoveToward(Velocity.Z, 0, Speed);
		}
		velocity.Z = (float)CalculateVelocity(delta, 17, Velocity.Z);
		GD.Print(velocity.Z);
		Velocity = velocity;
		MoveAndSlide();
	}
}


public class WheelSuspensionSpring
{
	public float Offset;
	public float Strength = 100;
	public float Damping = 10;

	public float GetForce(float velocity)
	{
		return (Offset * Strength) - (velocity * Damping);
	}
}


public class WheelInfo
{
	public WheelInfo(Node3D wheelTransform)
	{
		this.WheelTransform = wheelTransform;
	}
	public Node3D WheelTransform;
	public bool Hitting = false;
	public Vector3 SurfaceNormal;
	public Vector3 HitWorldLocation;
}
