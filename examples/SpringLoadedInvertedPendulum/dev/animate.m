% a test trajectory for the visualizer

for i = 1:numel(xtraj)
v.draw(t(i), xtraj(:, i));
end