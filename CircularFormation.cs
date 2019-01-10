using System.Collections.Generic;
using UnityEngine;
using UnityEngine.AI;

public class CircularFormation : IFormation {

    private const int DefaultRingsCount = 3;
    private const int BufferMultiplier = 3;

    private readonly float _actorSizeWithBuffer;
    private readonly int _startFromAngle;
    private readonly int _direction;
    private readonly int _rings;
    private readonly bool _breakWhenMinPointsOnSideReached;
    private readonly bool _halfCircle;
    private readonly bool _includeOrigin;

    public CircularFormation(
        float actorSize, 
        int startFromAngle, 
        FormationDirection direction, 
        bool breakWhenMinPointsOnSideReached, 
        bool halfCircle,
        bool includeOrigin) {

        _actorSizeWithBuffer = actorSize * BufferMultiplier;
        _startFromAngle = startFromAngle;
        _direction = (int)direction;
        _rings = DefaultRingsCount;
        _breakWhenMinPointsOnSideReached = breakWhenMinPointsOnSideReached;
        _halfCircle = halfCircle;
        _includeOrigin = includeOrigin;
    }

    public CircularFormation(
        float actorSize, 
        int startFromAngle, 
        FormationDirection direction, 
        int bufferMultiplier, 
        int ringsCount, 
        bool breakWhenMinPointsOnSideReached, 
        bool halfCircle,
        bool includeOrigin) {

        _actorSizeWithBuffer = actorSize * bufferMultiplier;
        _startFromAngle = startFromAngle;
        _direction = (int)direction;
        _rings = ringsCount;
        _breakWhenMinPointsOnSideReached = breakWhenMinPointsOnSideReached;
        _halfCircle = halfCircle;
        _includeOrigin = includeOrigin;
    }

    public IReadOnlyList<Vector3> CalculateMoves(Vector3 goal, Vector3 currentPosition, int minPointsOnSideOrQuarter) {
        var relativePos = goal - currentPosition;
        var localRotation = Quaternion.LookRotation(relativePos);
        var vectors = CalculateMoveVectors(goal, localRotation, _startFromAngle, minPointsOnSideOrQuarter);

        return vectors;
    }

    public Vector3 AdjustGoal(Vector3 point) {
        if (NavMesh.SamplePosition(point, out NavMeshHit hit, _actorSizeWithBuffer, NavMesh.AllAreas)) {
            return hit.position;
        }

        return Vector3.zero;
    }

    private IReadOnlyList<Vector3> CalculateMoveVectors(Vector3 origin, Quaternion localRotation, int startFromDegree, int minPointsOnSideOrQuarter) {
        var leftDegrees = Range(startFromDegree, _halfCircle ? 10 : 20, 10 * _direction);
        var rightDegrees = Range(-startFromDegree, _halfCircle ? 10 : 20, -10 * _direction);
        var leftPoints = FindPoints(origin, localRotation, leftDegrees, minPointsOnSideOrQuarter);
        var rightPoints = FindPoints(origin, localRotation, rightDegrees, minPointsOnSideOrQuarter);
        var result = new List<Vector3>();
        var longerList = leftPoints.Count > rightPoints.Count ? leftPoints : rightPoints;
        
        if (_includeOrigin) {
            var adjustedOrigin = AdjustGoal(origin);

            if (adjustedOrigin == Vector3.zero) {
                return result;
            }

            result.Add(adjustedOrigin);
        }

        for (var i = 0; i < longerList.Count; i++) {
            if (i < leftPoints.Count) {
                result.Add(leftPoints[i]);
            }
            if (i < rightPoints.Count) {
                result.Add(rightPoints[i]);
            }
        }

        return result;
    }

    private IEnumerable<int> Range(int start, int count, int step) {
        var range = new int[count];

        for (var i = 0; i < count; i++) {
            range[i] = start + i * step;
        }

        return range;
    }

    private List<Vector3> FindPoints(Vector3 origin, Quaternion localRotation, IEnumerable<int> degrees, int pointsCount) {
        var result = new List<Vector3>();

        for (var i = 0; i < _rings; i++) {
            foreach (var degree in degrees) {
                var vector = MoveVector(origin, localRotation, degree, i + 1);
                var closest = AdjustGoal(vector);

                if (closest == Vector3.zero) {
                    continue;
                }

                result.Add(closest);

                if (result.Count >= pointsCount && _breakWhenMinPointsOnSideReached) {
                    break;
                }
            }
        }

        return result;
    }

    private Vector3 MoveVector(Vector3 origin, Quaternion localRotation, int degree, int ring) {
        var teamRotation = Quaternion.AngleAxis(degree, Vector3.up) * localRotation;
        var nextPoint = origin + new Vector3(0, 0, _actorSizeWithBuffer * ring);
        var direction = nextPoint - origin;
        var rotatedDirection = teamRotation * direction;
        var movedVector = rotatedDirection + origin;

        return movedVector;
    }

}
