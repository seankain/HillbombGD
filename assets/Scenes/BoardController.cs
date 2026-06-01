using Godot;
using System;

/// <summary>
/// Arcade skateboard controller for downhill gameplay.
/// Lean-to-steer coupling derived from truck geometry:
///   ψ̇ = v · (tan λ_f + tan λ_r) · tan φ / L
/// Lean is driven by a first-order spring tracking player input.
/// Speed wobble injects sinusoidal perturbation above a threshold.
/// </summary>
public partial class BoardController : CharacterBody3D, IRespawnablePlayer
{
    [ExportGroup("Rider / Board")]
    [Export] public float Mass = 75f;               // kg, rider + board
    [Export] public float CenterOfMassHeight = 0.9f;// m, COM above ground

    [ExportGroup("Truck Geometry")]
    [Export] public float FrontPivotAngleDeg = 50f; // degrees from horizontal
    [Export] public float RearPivotAngleDeg = 50f;  // degrees from horizontal
    [Export] public float Wheelbase = 0.35f;        // m, front to rear truck contact

    [ExportGroup("Lean Control")]
    [Export] public float MaxLeanAngleDeg = 22f;    // degrees, rider lean limit
    [Export] public float LeanSpringRate = 25f;     // how snappy lean tracks input
    [Export] public float MinSteerSpeed = 3f;       // m/s, minimum effective speed for steering
    [Export] public float MaxSteerSpeed = 25f;      // m/s, cap speed used in yaw-rate calc

    [ExportGroup("Speed Wobble")]
    [Export] public float WobbleSpeedThreshold = 18f; // m/s, when wobble kicks in
    [Export] public float WobbleMaxAmplitude = 0.08f; // radians (~4.5 degrees)
    [Export] public float WobbleFrequencyRange = 8f;  // Hz base frequency

    [ExportGroup("Powerslide")]
    [Export] public float SlideYawAngleDeg = 75f;
    [Export] public float SlideBrakingCoefficient = 3f;
    [Export] public float SlideSteerFactor = 0.2f;
    [Export] public float SlideYawSnapSpeed = 10f;

    [ExportGroup("Aerodynamics and Friction")]
    [Export] public float AirDensity = 1.225f;      // kg/m³
    [Export] public float DragCoefficient = 1.0f;    // dimensionless
    [Export] public float FrontalArea = 0.5f;        // m²
    [Export] public float RollingResistance = 0.01f; // dimensionless

    [ExportGroup("Jump")]
    [Export] public float JumpSpeed = 5f;           // m/s vertical impulse

    [ExportGroup("Raycasts")]
    [Export] public RayCast3D FrontTruckRay;
    [Export] public RayCast3D RearTruckRay;

    // Physics state ───────────────────────────────────────────────────────────
    private float _speed;       // m/s, along heading
    private float _yaw;         // rad; forward direction = (sin ψ, 0, cos ψ)
    private float _leanAngle;   // φ rad; positive = lean right
    private float _leanRate;    // φ̇ rad/s
    private float _airVelY;     // m/s vertical when airborne
    private float _wobblePhase; // accumulated wobble oscillator phase
    private bool _isSliding;
    private float _slideYawOffset;
    private float _slideDirection;

    // Debug-readable state ────────────────────────────────────────────────────
    public bool Grounded { get; private set; }
    public bool IsSliding => _isSliding;
    public float LeanAngle => _leanAngle;
    public Vector3 CurrentSurfaceNormal { get; private set; }
    public float SlopeAngle { get; private set; }

    public event PlayerRespawnedEventHandler PlayerRespawned;

    public float gravity = ProjectSettings.GetSetting("physics/3d/default_gravity").AsSingle();

    private AnimationPlayer _animPlayer;

    // ─────────────────────────────────────────────────────────────────────────


    public override void _Ready()
    {
        _animPlayer = GetNodeOrNull<AnimationPlayer>("AnimationPlayer");
        _yaw = 0f;
    }

    public override void _Input(InputEvent @event)
    {
        if (@event.IsActionPressed("Reset"))
            Respawn();
    }

    public override void _PhysicsProcess(double delta)
    {
        float dt = (float)delta;
        bool grounded = IsOnFloor();
        Grounded = grounded;
        Vector3 normal = GetSurfaceNormal();
        CurrentSurfaceNormal = normal;

        // ── Truck pivot angles ────────────────────────────────────────────────
        float tanPivotF = Mathf.Tan(Mathf.DegToRad(FrontPivotAngleDeg));
        float tanPivotR = Mathf.Tan(Mathf.DegToRad(RearPivotAngleDeg));

        // ── Powerslide state ──────────────────────────────────────────────────
        float leanInput  = Input.GetAxis("Left", "Right");
        bool slideInput  = Input.IsActionPressed("SpeedRotate") && Mathf.Abs(leanInput) > 0.1f && grounded;

        if (slideInput && !_isSliding)
        {
            _isSliding = true;
            _slideDirection = Mathf.Sign(leanInput);
        }
        else if (!slideInput)
        {
            _isSliding = false;
        }

        float slideTargetYaw = _isSliding ? _slideDirection * Mathf.DegToRad(SlideYawAngleDeg) : 0f;
        _slideYawOffset = Mathf.Lerp(_slideYawOffset, slideTargetYaw, SlideYawSnapSpeed * dt);

        // ── Lean-to-steer coupling ────────────────────────────────────────────
        float steerFactor = _isSliding ? SlideSteerFactor : 1f;
        float steerSpeed = Mathf.Clamp(_speed, MinSteerSpeed, MaxSteerSpeed);
        float tanLean    = Mathf.Tan(_leanAngle);
        float yawRate    = -steerSpeed * (tanPivotF + tanPivotR) * tanLean / Wheelbase * steerFactor;

        // ── Lean control ──────────────────────────────────────────────────────
        float targetLean = leanInput * Mathf.DegToRad(MaxLeanAngleDeg);

        if (grounded)
        {
            _leanRate = LeanSpringRate * (targetLean - _leanAngle);
            _leanAngle += _leanRate * dt;
            _leanAngle = Mathf.Clamp(_leanAngle,
                -Mathf.DegToRad(MaxLeanAngleDeg),
                Mathf.DegToRad(MaxLeanAngleDeg));

            // Speed wobble: sinusoidal perturbation above threshold
            if (_speed > WobbleSpeedThreshold)
            {
                float wobbleIntensity = Mathf.Clamp((_speed - WobbleSpeedThreshold) / 15f, 0f, 1f);
                _wobblePhase += WobbleFrequencyRange * Mathf.Tau * dt;
                _leanAngle += wobbleIntensity * WobbleMaxAmplitude * Mathf.Sin(_wobblePhase);
            }
        }
        else
        {
            _leanRate  *= Mathf.Exp(-8f * dt);
            _leanAngle  = Mathf.Lerp(_leanAngle, 0f, 4f * dt);
        }

        // ── Forward speed ─────────────────────────────────────────────────────
        float slopeAngle = ComputeSlopeAngle(normal);
        SlopeAngle = slopeAngle;

        if (grounded)
        {
            float gravComp   = gravity * Mathf.Sin(slopeAngle);
            float dragComp   = 0.5f * AirDensity * DragCoefficient * FrontalArea
                               * _speed * Mathf.Abs(_speed) / Mass;
            float rollComp   = RollingResistance * gravity * Mathf.Cos(slopeAngle);
            float slideComp  = _isSliding ? SlideBrakingCoefficient * _speed : 0f;
            _speed += (gravComp - dragComp - rollComp - slideComp) * dt;
        }
        else
        {
            float dragComp = 0.5f * AirDensity * DragCoefficient * FrontalArea
                             * _speed * Mathf.Abs(_speed) / Mass;
            _speed -= dragComp * dt;
        }

        // ── Integrate yaw ─────────────────────────────────────────────────────
        if (grounded)
            _yaw += yawRate * dt;

        // ── Jump ──────────────────────────────────────────────────────────────
        bool jumping = false;
        if (Input.IsActionJustPressed("Jump") && grounded)
        {
            _airVelY = JumpSpeed;
            jumping = true;
        }

        // ── Build velocity for CharacterBody3D ────────────────────────────────
        Vector3 fwdWorld = new Vector3(Mathf.Sin(_yaw), 0f, Mathf.Cos(_yaw));

        Vector3 velocity;
        if (grounded && !jumping)
        {
            Vector3 fwdSlope = ProjectOntoPlane(fwdWorld, normal);
            velocity = fwdSlope * _speed;
        }
        else
        {
            if (!jumping)
                _airVelY -= gravity * dt;
            velocity   = fwdWorld * _speed;
            velocity.Y = _airVelY;
        }

        Velocity = velocity;
        MoveAndSlide();

        // _speed is purely physics-driven; do NOT sync it back from Velocity.
        // Only track vertical velocity when airborne for correct jump arcs.
        if (!grounded && !jumping)
            _airVelY = Velocity.Y;

        // ── Orient board to surface + lean ────────────────────────────────────
        UpdateOrientation(normal, dt);
        UpdateLeanAnimation();
    }

    // ── Helpers ───────────────────────────────────────────────────────────────

    private Vector3 GetSurfaceNormal()
    {
        bool fHit = FrontTruckRay != null && FrontTruckRay.IsColliding();
        bool rHit = RearTruckRay  != null && RearTruckRay.IsColliding();
        if (fHit && rHit)
            return ((FrontTruckRay.GetCollisionNormal() + RearTruckRay.GetCollisionNormal()) * 0.5f).Normalized();
        if (fHit) return FrontTruckRay.GetCollisionNormal();
        if (rHit) return RearTruckRay.GetCollisionNormal();
        if (IsOnFloor()) return GetFloorNormal();
        return Vector3.Up;
    }

    private float ComputeSlopeAngle(Vector3 normal)
    {
        Vector3 fwdH  = new Vector3(Mathf.Sin(_yaw), 0f, Mathf.Cos(_yaw));
        Vector3 right = fwdH.Cross(normal);
        if (right.LengthSquared() < 1e-6f) return 0f;
        Vector3 fwdSlope = normal.Cross(right.Normalized()).Normalized();
        return Mathf.Asin(Mathf.Clamp(-fwdSlope.Y, -1f, 1f));
    }

    private static Vector3 ProjectOntoPlane(Vector3 v, Vector3 normal)
    {
        Vector3 p = v - v.Dot(normal) * normal;
        return p.LengthSquared() > 1e-6f ? p.Normalized() : v;
    }

    private void UpdateOrientation(Vector3 normal, float dt)
    {
        float visualYaw = _yaw + _slideYawOffset;
        Vector3 fwdH  = new Vector3(Mathf.Sin(visualYaw), 0f, Mathf.Cos(visualYaw));
        Vector3 right = fwdH.Cross(normal);
        if (right.LengthSquared() < 1e-4f) return;
        right = right.Normalized();
        Vector3 up  = normal;
        Vector3 fwd = up.Cross(right).Normalized();

        var leanQuat    = new Quaternion(fwd, _leanAngle);
        var targetBasis = new Basis(leanQuat * right, leanQuat * up, -fwd);
        var smoothed    = GlobalTransform.Basis.Slerp(targetBasis, Mathf.Min(1f, 12f * dt));
        GlobalTransform = new Transform3D(smoothed, GlobalTransform.Origin);
    }

    private void UpdateLeanAnimation()
    {
        if (_animPlayer == null) return;
        if (_leanAngle < -0.05f)
            _animPlayer.Play("TurnLeft");
        else if (_leanAngle > 0.05f)
            _animPlayer.Play("TurnRight");
    }

    private void Respawn()
    {
        var respawn = GetTree().GetNodesInGroup("Respawn")[0] as Node3D;
        GlobalTransform = new Transform3D(respawn.GlobalTransform.Basis, respawn.GlobalPosition);
        PlayerRespawned?.Invoke(this, new PlayerRespawnArgs());
        Velocity    = Vector3.Zero;
        _speed      = 0f;
        _leanAngle  = 0f;
        _leanRate   = 0f;
        _yaw        = 0f;
        _airVelY    = 0f;
        _wobblePhase = 0f;
        _isSliding = false;
        _slideYawOffset = 0f;
        _slideDirection = 0f;
    }
}


public class WheelSuspensionSpring
{
    public float Offset;
    public float Strength = 100;
    public float Damping = 10;
    public float GetForce(float velocity) => (Offset * Strength) - (velocity * Damping);
}


public class WheelInfo
{
    public WheelInfo(Node3D wheelTransform) => WheelTransform = wheelTransform;
    public Node3D WheelTransform;
    public bool Hitting;
    public Vector3 SurfaceNormal;
    public Vector3 HitWorldLocation;
}
