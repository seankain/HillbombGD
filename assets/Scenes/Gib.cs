using Godot;
using System;
using System.ComponentModel;
using System.Security.Cryptography.X509Certificates;

public partial class Gib : RigidBody3D
{
    private bool needsReset = false;
    private Transform3D ComponentStartTransform;

    public override void _Ready()
    {
        this.ComponentStartTransform = new Transform3D(this.Transform.Basis, GetParentNode3D().Transform.Origin);
    }

    public void Reset()
    {
        this.needsReset = true;
        this.ComponentStartTransform = new Transform3D(this.ComponentStartTransform.Basis, GetParentNode3D().GlobalTransform.Origin);
    }

    public void Disable()
    {
        this.ProcessMode = ProcessModeEnum.Disabled;
        this.SetPhysicsProcess(false);
    }

    public void Enable()
    {
        this.ProcessMode = ProcessModeEnum.Always;
        this.SetPhysicsProcess(true);
    }


    public override void _PhysicsProcess(double delta)
    {
        if (needsReset)
        {
            this.Sleeping = true;
            this.LinearVelocity = Vector3.Zero;
            this.AngularVelocity = Vector3.Zero;
            this.GlobalTransform = new Transform3D(this.ComponentStartTransform.Basis, this.ComponentStartTransform.Origin);
            this.Sleeping = false;
            needsReset = false;
        }
        base._PhysicsProcess(delta);
    }

}
