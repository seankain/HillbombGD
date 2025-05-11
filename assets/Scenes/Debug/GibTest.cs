using Godot;
using System;

public partial class GibTest : Node3D
{

    [Export]
    public Gibsplosion GibSceneInstance;

    [Export]
    public SpinBox AngularVelocityX;

    [Export]
    public SpinBox AngularVelocityY;

    [Export]
    public SpinBox AngularVelocityZ;

    [Export]
    public SpinBox LinearVelocityX;

    [Export]
    public SpinBox LinearVelocityY;

    [Export]
    public SpinBox LinearVelocityZ;

    [Export]
    public SpinBox ForceMultiplier;

    public void GibButtonClicked()
    {
        GibSceneInstance.Gibsplode(GetForceMultiplier(), GetLinearVelocity(), GetAngularVelocity());
    }

    public void ResetButtonClicked()
    {
        GibSceneInstance.Ungib();
    }

    private Vector3 GetLinearVelocity()
    {
        return new Vector3((float)LinearVelocityX.Value, (float)LinearVelocityY.Value, (float)LinearVelocityZ.Value);
    }
    private Vector3 GetAngularVelocity()
    {
        return new Vector3((float)AngularVelocityX.Value, (float)AngularVelocityY.Value, (float)AngularVelocityZ.Value);
    }

    private float GetForceMultiplier()
    {
        return (float)ForceMultiplier.Value;
    }
}
