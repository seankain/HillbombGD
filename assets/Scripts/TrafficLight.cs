using Godot;
using System;

public enum TrafficLightState
{
	Green,
	Yellow,
	Red
}

public partial class TrafficLight : Node3D
{
	public TrafficLightState State { get; private set; } = TrafficLightState.Red;

	private MeshInstance3D _mesh;
	private StandardMaterial3D _material;

	public override void _Ready()
	{
		_mesh = GetChildByType<MeshInstance3D>();
		if (_mesh != null)
		{
			_material = new StandardMaterial3D();
			_mesh.MaterialOverride = _material;
		}
		ApplyColor();
	}

	public void SetState(TrafficLightState state)
	{
		State = state;
		ApplyColor();
	}

	private void ApplyColor()
	{
		if (_material == null)
			return;

		_material.EmissionEnabled = true;
		switch (State)
		{
			case TrafficLightState.Green:
				_material.AlbedoColor = new Color(0.1f, 0.8f, 0.1f);
				_material.Emission = new Color(0.1f, 0.8f, 0.1f);
				break;
			case TrafficLightState.Yellow:
				_material.AlbedoColor = new Color(0.9f, 0.8f, 0.1f);
				_material.Emission = new Color(0.9f, 0.8f, 0.1f);
				break;
			case TrafficLightState.Red:
				_material.AlbedoColor = new Color(0.9f, 0.1f, 0.1f);
				_material.Emission = new Color(0.9f, 0.1f, 0.1f);
				break;
		}
	}

	private T GetChildByType<T>() where T : Node
	{
		for (int i = 0; i < GetChildCount(); i++)
		{
			if (GetChild(i) is T t)
				return t;
		}
		return null;
	}
}
