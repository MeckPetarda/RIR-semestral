function isSuccesfullyPassed = CheckWayPointTrack(bodyXYZPosition, actualTime, timeForWaypointPasage, wayPoints, positionTolerance)
    % Function that determines whether the quadcopter will pass a specified point in a given time within a given tolerance

    isSuccesfullyPassed = false;
    index = 0;

    for i = 1:length(timeForWaypointPasage)
      referenceTime = timeForWaypointPasage(i);

      if (referenceTime > actualTime)
        break
      end

      index = i;
    end

    if (index == 0)
      return
    end

    currentWaypoint = wayPoints(index, :);
    positionVector = [bodyXYZPosition.X, bodyXYZPosition.Y, bodyXYZPosition.Z];

    diff = currentWaypoint - positionVector;
    diffMagnitude = sqrt(diff(1)^2 + diff(2)^2 + diff(3)^2);

    if (diffMagnitude > positionTolerance)
      isSuccesfullyPassed = true;
      return
    end
end
