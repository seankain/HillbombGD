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

    [ExportGroup("Airborne Control")]
    [Export] public float AirSteerRate = 1.5f;      // rad/s, yaw adjustment from Left/Right in air
    [Export] public float AirSpinRate = 6f;          // rad/s, spin speed when powerslide + direction in air

    [ExportGroup("Jump")]
    [Export] public float JumpSpeed = 5f;           // m/s vertical impulse
    [Export] public float FallGravityMultiplier = 2.5f; // extra gravity when falling

    [ExportGroup("Crash Bail")]
    [Export] public float CrashImpactThreshold = 450f; // closing speed × mass (kg·m/s) to count as a crash
    [Export] public float CrashBailDuration = 1.5f;    // s, time with no control before respawn
    [Export] public float CrashWallNormalMaxY = 0.7f;  // ignore impacts whose normal is more vertical than this (floors/landings)
    [Export] public float CrashGroundFriction = 2.5f;  // how quickly the bailing board bleeds off momentum on the ground

    [ExportGroup("Grind")]
    [Export] public float GrindMinEntrySpeed = 4f;       // m/s, minimum speed to latch onto a rail
    [Export] public float GrindMaxAlignmentAngleDeg = 45f;// max angle between travel and rail to enter
    [Export] public float GrindHeightOffset = 0.12f;     // m, how far the board sits above the rail line
    [Export] public float GrindFriction = 0.5f;          // m/s², speed bleed while grinding (flat rail)
    [Export] public float GrindMinSpeed = 3f;            // m/s, grind never slows below this
    [Export] public float GrindBalanceInstabilityDeg = 200f; // deg/s² per rad of tilt; how fast you tip over
    [Export] public float GrindBalanceControl = 6f;      // rad/s² of corrective accel from lean input
    [Export] public float GrindBalanceDamping = 2f;      // 1/s, balance velocity damping
    [Export] public float GrindBalanceNoiseDeg = 35f;    // deg/s², random destabilizing nudge
    [Export] public float GrindBalanceLimitDeg = 22f;    // deg, tilt beyond this bails the grind
    [Export] public float GrindReentryCooldown = 0.4f;   // s, lockout after leaving a rail

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
    private float _airSpinOffset;
    private bool _isCrashed;     // true during a crash bail (no control until respawn)
    private float _crashTimer;   // s remaining in the current bail
    private bool _isGrinding;    // true while locked onto a grind rail
    private Path3D _grindPath;   // the rail's path being ridden
    private float _grindOffset;  // baked distance along the rail curve
    private int _grindDir;       // +1/-1, travel direction along the curve's natural offset
    private float _grindBalance; // rad, board tilt while grinding (0 = balanced)
    private float _grindBalanceVel; // rad/s, rate of tilt change
    private float _grindCooldown;// s, re-entry lockout after leaving a rail

    // Debug-readable state ────────────────────────────────────────────────────
    public bool Grounded { get; private set; }
    public bool IsSliding => _isSliding;
    public bool IsCrashed => _isCrashed;
    public bool IsGrinding => _isGrinding;
    public float LeanAngle => _leanAngle;
    public Vector3 CurrentSurfaceNormal { get; private set; }
    public float SlopeAngle { get; private set; }

    public event PlayerRespawnedEventHandler PlayerRespawned;

    public float gravity = ProjectSettings.GetSetting("physics/3d/default_gravity").AsSingle();

    private AnimationPlayer _animPlayer;
    private CameraController _camera;

    // ─────────────────────────────────────────────────────────────────────────


    public override void _Ready()
    {
        _animPlayer = GetNodeOrNull<AnimationPlayer>("AnimationPlayer");
        _camera = GetNodeOrNull<CameraController>("CameraController");
        _yaw = 0f;
    }

    public override void _Input(InputEvent @event)
    {
        // No board control while bailing from a crash.
        if (_isCrashed)
            return;

        if (@event.IsActionPressed("Reset"))
            Respawn();
    }

    public override void _PhysicsProcess(double delta)
    {
        float dt = (float)delta;

        // While bailing from a crash the player has no control over board or
        // camera; just let the board settle until the bail timer elapses.
        if (_isCrashed)
        {
            UpdateCrash(dt);
            return;
        }

        // While grinding the board is locked to the rail path; balance + jump are
        // the only inputs, handled entirely in UpdateGrind.
        if (_isGrinding)
        {
            UpdateGrind(dt);
            return;
        }

        if (_grindCooldown > 0f)
            _grindCooldown -= dt;

        bool grounded = IsOnFloor();
        Grounded = grounded;
        Vector3 normal = GetSurfaceNormal();
        CurrentSurfaceNormal = normal;

        // ── Truck pivot angles ────────────────────────────────────────────────
        if (grounded && _airSpinOffset != 0f)
            _airSpinOffset = 0f;

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

            bool airSlideInput = Input.IsActionPressed("SpeedRotate") && Mathf.Abs(leanInput) > 0.1f;
            if (airSlideInput)
                _airSpinOffset -= leanInput * AirSpinRate * dt;
            else
                _yaw -= leanInput * AirSteerRate * dt;
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
            {
                float gravMult = _airVelY < 0f ? FallGravityMultiplier : 1f;
                _airVelY -= gravity * gravMult * dt;
            }
            velocity   = fwdWorld * _speed;
            velocity.Y = _airVelY;
        }

        Velocity = velocity;
        Vector3 preMoveVelocity = velocity;
        MoveAndSlide();

        // Latching onto a rail takes priority over crashing: a grindable contact
        // becomes a grind instead of a bail when approached along the rail.
        if (_grindCooldown <= 0f && TryEnterGrind(preMoveVelocity))
            return;

        // A hard enough collision this frame ends the run in a crash bail.
        if (CheckForCrash(preMoveVelocity))
            return;

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
        float visualYaw = _yaw + _slideYawOffset + _airSpinOffset;
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

    /// <summary>
    /// Inspects this frame's slide collisions and, if any impact exceeds the
    /// crash threshold, enters the crash bail state. Floor/landing impacts
    /// (near-vertical normals) are ignored so only head-on hits against walls,
    /// cars, and obstacles can crash the rider.
    /// </summary>
    /// <param name="preMoveVelocity">Velocity before MoveAndSlide deflected it.</param>
    /// <returns>True if a crash was triggered this frame.</returns>
    private bool CheckForCrash(Vector3 preMoveVelocity)
    {
        int count = GetSlideCollisionCount();
        for (int i = 0; i < count; i++)
        {
            KinematicCollision3D collision = GetSlideCollision(i);
            Vector3 normal = collision.GetNormal();

            // Skip floor/landing contacts; only roughly-horizontal impacts crash.
            if (Mathf.Abs(normal.Y) > CrashWallNormalMaxY)
                continue;

            // Closing speed into the surface, relative to a (possibly moving) collider.
            Vector3 relativeVelocity = preMoveVelocity - collision.GetColliderVelocity();
            float closingSpeed = -relativeVelocity.Dot(normal);
            if (closingSpeed <= 0f)
                continue;

            float impactForce = closingSpeed * Mass;
            if (impactForce >= CrashImpactThreshold)
            {
                EnterCrash();
                return true;
            }
        }
        return false;
    }

    private void EnterCrash()
    {
        _isCrashed = true;
        _crashTimer = CrashBailDuration;
        _isSliding = false;

        // Lock the camera so the player loses camera control during the bail.
        if (_camera != null)
            _camera.Frozen = true;
    }

    /// <summary>
    /// Runs while crashed: no player input is read. The board keeps its
    /// post-impact momentum, falls under gravity, and bleeds off speed on the
    /// ground until the bail timer elapses, at which point the rider respawns.
    /// </summary>
    private void UpdateCrash(float dt)
    {
        Vector3 velocity = Velocity;
        if (IsOnFloor())
        {
            velocity.X = Mathf.Lerp(velocity.X, 0f, CrashGroundFriction * dt);
            velocity.Z = Mathf.Lerp(velocity.Z, 0f, CrashGroundFriction * dt);
        }
        else
        {
            velocity.Y -= gravity * dt;
        }

        Velocity = velocity;
        MoveAndSlide();

        _crashTimer -= dt;
        if (_crashTimer <= 0f)
            Respawn();
    }

    // ── Grind ───────────────────────────────────────────────────────────────────

    /// <summary>
    /// Inspects this frame's slide collisions for a grindable rail and, if the
    /// board has enough speed and is travelling roughly along the rail, latches
    /// onto it. Grindable geometry is tagged with the "Grindable" group and
    /// carries a <see cref="Path3D"/> (via a "grind_path" NodePath metadata, by
    /// being a Path3D itself, or as a descendant) describing the rail line.
    /// </summary>
    /// <param name="preMoveVelocity">Velocity before MoveAndSlide deflected it.</param>
    /// <returns>True if a grind was entered this frame.</returns>
    private bool TryEnterGrind(Vector3 preMoveVelocity)
    {
        if (_speed < GrindMinEntrySpeed)
            return false;

        int count = GetSlideCollisionCount();
        for (int i = 0; i < count; i++)
        {
            Path3D path = FindGrindPath(GetSlideCollision(i).GetCollider());
            if (path?.Curve == null || path.Curve.GetBakedLength() <= 0f)
                continue;

            // Offset of the closest point on the rail to the board.
            Vector3 local = path.ToLocal(GlobalPosition);
            float offset = path.Curve.GetClosestOffset(local);

            // Rail tangent (world) in its natural increasing-offset direction.
            Vector3 tangent = GrindNaturalTangent(path, offset);
            Vector3 tangentH = new Vector3(tangent.X, 0f, tangent.Z);
            Vector3 travelH  = new Vector3(preMoveVelocity.X, 0f, preMoveVelocity.Z);
            if (tangentH.LengthSquared() < 1e-6f || travelH.LengthSquared() < 1e-6f)
                continue;

            tangentH = tangentH.Normalized();
            travelH  = travelH.Normalized();

            // Must be approaching roughly along the rail, not across it.
            float alignment = travelH.Dot(tangentH);
            if (Mathf.Abs(alignment) < Mathf.Cos(Mathf.DegToRad(GrindMaxAlignmentAngleDeg)))
                continue;

            EnterGrind(path, offset, alignment >= 0f ? 1 : -1);
            return true;
        }
        return false;
    }

    private void EnterGrind(Path3D path, float offset, int dir)
    {
        _isGrinding   = true;
        _isSliding    = false;
        _grindPath    = path;
        _grindOffset  = offset;
        _grindDir     = dir;
        _grindBalance = 0f;
        _grindBalanceVel = 0f;
        _airVelY      = 0f;
        _slideYawOffset = 0f;
        _airSpinOffset = 0f;
        _speed        = Mathf.Max(_speed, GrindMinSpeed);
    }

    /// <summary>
    /// Runs each frame while grinding. The board is pinned to the rail curve and
    /// driven along its tangent, accelerating downhill and bleeding a little
    /// friction. A balance mini-game runs in parallel: lean to counter the tilt,
    /// or bail when it exceeds the limit. Exits on jump, reaching a rail end, or
    /// losing balance.
    /// </summary>
    private void UpdateGrind(float dt)
    {
        Curve3D curve = _grindPath?.Curve;
        if (curve == null || curve.GetBakedLength() <= 0f)
        {
            ExitGrind(false);
            return;
        }

        float length = curve.GetBakedLength();
        Vector3 tangent = GrindNaturalTangent(_grindPath, _grindOffset) * _grindDir;

        // ── Speed along the rail: gravity downhill minus a little friction. ──────
        float slope = Mathf.Asin(Mathf.Clamp(-tangent.Y, -1f, 1f));
        _speed += (gravity * Mathf.Sin(slope) - GrindFriction) * dt;
        _speed = Mathf.Max(_speed, GrindMinSpeed);

        // ── Balance mini-game ───────────────────────────────────────────────────
        float leanInput = Input.GetAxis("Left", "Right");
        float instability = Mathf.DegToRad(GrindBalanceInstabilityDeg);
        float noise = Mathf.DegToRad(GrindBalanceNoiseDeg);
        _grindBalanceVel += _grindBalance * instability * dt;          // tips over faster as it leans
        _grindBalanceVel += leanInput * GrindBalanceControl * dt;      // player correction
        _grindBalanceVel += (GD.Randf() * 2f - 1f) * noise * dt;       // random wobble
        _grindBalanceVel *= Mathf.Exp(-GrindBalanceDamping * dt);
        _grindBalance += _grindBalanceVel * dt;

        if (Mathf.Abs(_grindBalance) > Mathf.DegToRad(GrindBalanceLimitDeg))
        {
            BailGrind();
            return;
        }

        // ── Jump off the rail ───────────────────────────────────────────────────
        if (Input.IsActionJustPressed("Jump"))
        {
            ExitGrind(true);
            return;
        }

        // ── Advance along the rail; leaving either end ends the grind. ──────────
        _grindOffset += _speed * _grindDir * dt;
        if (_grindOffset <= 0f || _grindOffset >= length)
        {
            _grindOffset = Mathf.Clamp(_grindOffset, 0f, length);
            ExitGrind(false);
            return;
        }

        // ── Pin the board to the rail and orient it along the tangent. ──────────
        Vector3 railPoint = _grindPath.ToGlobal(curve.SampleBaked(_grindOffset));
        GlobalPosition = railPoint + Vector3.Up * GrindHeightOffset;

        _yaw = Mathf.Atan2(tangent.X, tangent.Z);
        _leanAngle = _grindBalance;
        Velocity = new Vector3(tangent.X, tangent.Y, tangent.Z) * _speed;

        UpdateOrientation(Vector3.Up, dt);
    }

    /// <summary>Leaves the rail, converting rail momentum back into the free-movement state.</summary>
    private void ExitGrind(bool jumped)
    {
        Vector3 travel = GrindNaturalTangent(_grindPath, _grindOffset) * _grindDir;
        _yaw     = Mathf.Atan2(travel.X, travel.Z);
        _airVelY = travel.Y * _speed;
        if (jumped)
            _airVelY += JumpSpeed;

        _isGrinding = false;
        _grindPath = null;
        _grindBalance = 0f;
        _grindBalanceVel = 0f;
        _grindCooldown = GrindReentryCooldown;
    }

    /// <summary>Lost balance: drop grind state and fall into a crash bail.</summary>
    private void BailGrind()
    {
        _isGrinding = false;
        _grindPath = null;
        _grindBalance = 0f;
        _grindBalanceVel = 0f;
        _grindCooldown = GrindReentryCooldown;
        EnterCrash();
    }

    /// <summary>World-space tangent of a rail curve at a baked offset, in the curve's natural direction.</summary>
    private static Vector3 GrindNaturalTangent(Path3D path, float offset)
    {
        Curve3D curve = path.Curve;
        float length = curve.GetBakedLength();
        const float eps = 0.05f;
        Vector3 a = path.ToGlobal(curve.SampleBaked(Mathf.Clamp(offset - eps, 0f, length)));
        Vector3 b = path.ToGlobal(curve.SampleBaked(Mathf.Clamp(offset + eps, 0f, length)));
        Vector3 t = b - a;
        return t.LengthSquared() > 1e-6f ? t.Normalized() : Vector3.Forward;
    }

    /// <summary>
    /// Resolves the grind <see cref="Path3D"/> for a collider, or null if the
    /// collider is not part of a grindable. Climbs to the nearest ancestor in the
    /// "Grindable" group, then resolves its rail path.
    /// </summary>
    private static Path3D FindGrindPath(GodotObject colliderObj)
    {
        if (colliderObj is not Node node)
            return null;

        Node grindable = node;
        while (grindable != null && !grindable.IsInGroup("Grindable"))
            grindable = grindable.GetParent();
        if (grindable == null)
            return null;

        if (grindable.HasMeta("grind_path"))
        {
            var np = grindable.GetMeta("grind_path").As<NodePath>();
            var p = grindable.GetNodeOrNull<Path3D>(np);
            if (p != null)
                return p;
        }

        if (grindable is Path3D self)
            return self;

        return FindFirstPath3D(grindable);
    }

    private static Path3D FindFirstPath3D(Node node)
    {
        foreach (Node child in node.GetChildren())
        {
            if (child is Path3D path)
                return path;
            Path3D nested = FindFirstPath3D(child);
            if (nested != null)
                return nested;
        }
        return null;
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
        _airSpinOffset = 0f;
        _isCrashed = false;
        _crashTimer = 0f;
        _isGrinding = false;
        _grindPath = null;
        _grindOffset = 0f;
        _grindDir = 0;
        _grindBalance = 0f;
        _grindBalanceVel = 0f;
        _grindCooldown = 0f;

        // Restore camera tracking after the bail.
        if (_camera != null)
            _camera.Frozen = false;
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
