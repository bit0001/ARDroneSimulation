function plotTrajectoryComparison(vectors, labelX, labelY, labelZ, ...
    legend1, legend2, legend3, strTitle)
    
    if size(vectors, 2) == 6
        plot3(vectors(:, 1), vectors(:, 2), vectors(:, 3), vectors(:, 4), ...
            vectors(:, 5), vectors(:, 6), 'r--');
        legend(legend1, legend2, 'Location','best');
    else
        plot3(vectors(:, 1), vectors(:, 2), vectors(:, 3), vectors(:, 4), ...
        vectors(:, 5), vectors(:, 6), 'r--', vectors(:, 7), ...
        vectors(:, 8), vectors(:, 9), 'g--');
        legend(legend1, legend2, legend3,'Location','best');
    end

    xlabel(labelX)
    ylabel(labelY)
    zlabel(labelZ)
    title(strTitle);
    grid on
    axis square
end