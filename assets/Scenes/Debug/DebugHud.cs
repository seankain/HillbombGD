using Godot;

public partial class DebugHud : Control
{
    private BoardController _board;

    private Label _frontRayVal;
    private Label _rearRayVal;
    private Label _groundedVal;
    private Label _velocityVal;
    private Label _leanVal;
    private Label _surfaceNormalVal;
    private Label _slopeAngleVal;

    public override void _Ready()
    {
        _board = GetTree().GetFirstNodeInGroup("Player") as BoardController
              ?? GetNode<BoardController>("../BoardController");

        _frontRayVal      = GetNode<Label>("VBoxContainer/FrontRayHBox/FrontRayValue");
        _rearRayVal       = GetNode<Label>("VBoxContainer/RearRayHBox/RearRayValue");
        _groundedVal      = GetNode<Label>("VBoxContainer/GroundedHBox/GroundedValue");
        _velocityVal      = GetNode<Label>("VBoxContainer/VelocityHBox/VelocityValue");
        _leanVal          = GetNode<Label>("VBoxContainer/LeanHBox/LeanValue");
        _surfaceNormalVal = GetNode<Label>("VBoxContainer/SurfaceNormalHBox/SurfaceNormalValue");
        _slopeAngleVal    = GetNode<Label>("VBoxContainer/SlopeAngleHBox/SlopeAngleValue");
    }

    public override void _Process(double delta)
    {
        if (_board == null) return;

        bool frontHit = _board.FrontTruckRay != null && _board.FrontTruckRay.IsColliding();
        bool rearHit  = _board.RearTruckRay  != null && _board.RearTruckRay.IsColliding();

        _frontRayVal.Text = frontHit ? "HIT" : "MISS";
        _rearRayVal.Text  = rearHit  ? "HIT" : "MISS";
        _groundedVal.Text = _board.Grounded ? "YES" : "NO";

        Vector3 vel = _board.Velocity;
        _velocityVal.Text = $"({vel.X:F2}, {vel.Y:F2}, {vel.Z:F2})  |{vel.Length():F2}| m/s";

        float leanDeg = Mathf.RadToDeg(_board.LeanAngle);
        _leanVal.Text = $"{leanDeg:F1}°";

        if (_board.Grounded)
        {
            Vector3 n = _board.CurrentSurfaceNormal;
            _surfaceNormalVal.Text = $"({n.X:F3}, {n.Y:F3}, {n.Z:F3})";
            _slopeAngleVal.Text = $"{Mathf.RadToDeg(_board.SlopeAngle):F1}°";
        }
        else
        {
            _surfaceNormalVal.Text = "N/A (airborne)";
            _slopeAngleVal.Text = "N/A (airborne)";
        }
    }
}
