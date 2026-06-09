using Godot;
using System;

public partial class TrafficLightController : Node3D
{
	[Export]
	public TrafficLight StraightOutgoing;
	[Export]
	public TrafficLight StraightIncoming;
	[Export]
	public TrafficLight CrossRight;
	[Export]
	public TrafficLight CrossLeft;

	[Export]
	public float GreenDuration = 8.0f;
	[Export]
	public float YellowDuration = 2.0f;
	[Export]
	public float AllRedDuration = 1.0f;

	private enum Phase
	{
		StraightGreen,
		StraightYellow,
		AllRedBeforeCross,
		CrossGreen,
		CrossYellow,
		AllRedBeforeStraight
	}

	private Phase _currentPhase = Phase.StraightGreen;
	private float _phaseTimer;

	public override void _Ready()
	{
		_phaseTimer = GreenDuration;
		ApplyPhase();
	}

	public override void _Process(double delta)
	{
		_phaseTimer -= (float)delta;
		if (_phaseTimer <= 0f)
			AdvancePhase();
	}

	private void AdvancePhase()
	{
		switch (_currentPhase)
		{
			case Phase.StraightGreen:
				_currentPhase = Phase.StraightYellow;
				_phaseTimer = YellowDuration;
				break;
			case Phase.StraightYellow:
				_currentPhase = Phase.AllRedBeforeCross;
				_phaseTimer = AllRedDuration;
				break;
			case Phase.AllRedBeforeCross:
				_currentPhase = Phase.CrossGreen;
				_phaseTimer = GreenDuration;
				break;
			case Phase.CrossGreen:
				_currentPhase = Phase.CrossYellow;
				_phaseTimer = YellowDuration;
				break;
			case Phase.CrossYellow:
				_currentPhase = Phase.AllRedBeforeStraight;
				_phaseTimer = AllRedDuration;
				break;
			case Phase.AllRedBeforeStraight:
				_currentPhase = Phase.StraightGreen;
				_phaseTimer = GreenDuration;
				break;
		}
		ApplyPhase();
	}

	private void ApplyPhase()
	{
		switch (_currentPhase)
		{
			case Phase.StraightGreen:
				SetStraight(TrafficLightState.Green);
				SetCross(TrafficLightState.Red);
				break;
			case Phase.StraightYellow:
				SetStraight(TrafficLightState.Yellow);
				SetCross(TrafficLightState.Red);
				break;
			case Phase.AllRedBeforeCross:
			case Phase.AllRedBeforeStraight:
				SetStraight(TrafficLightState.Red);
				SetCross(TrafficLightState.Red);
				break;
			case Phase.CrossGreen:
				SetStraight(TrafficLightState.Red);
				SetCross(TrafficLightState.Green);
				break;
			case Phase.CrossYellow:
				SetStraight(TrafficLightState.Red);
				SetCross(TrafficLightState.Yellow);
				break;
		}
	}

	private void SetStraight(TrafficLightState state)
	{
		StraightOutgoing?.SetState(state);
		StraightIncoming?.SetState(state);
	}

	private void SetCross(TrafficLightState state)
	{
		CrossRight?.SetState(state);
		CrossLeft?.SetState(state);
	}

	public TrafficLightState GetStateForPath(Path3D path)
	{
		string name = path.Name;
		if (name.Contains("ThruPath") || name.Contains("Thru"))
		{
			return StraightOutgoing?.State ?? TrafficLightState.Green;
		}
		if (name.Contains("Intersection"))
		{
			return CrossRight?.State ?? TrafficLightState.Green;
		}
		return TrafficLightState.Green;
	}
}
