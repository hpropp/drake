% Run airplane example
Vsall = runReachability;

% Check to see that all Vs are greater than 1
if ~all(Vsall > 1)
    error('Reachability funnel is violated. Something is wrong!')
else
    disp('Passed test: All trajectories stay inside funnel')
end