%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function plotAxisComparison(vectors, legend1, legend2, legend3, labelX, labelY)
    vectorsSize = size(vectors);
    if (vectorsSize(2) == 3)
        plot(vectors(:, 1), vectors(:, 2), vectors(:, 1), ...
            vectors(:, 3), 'r--');
        legend(legend1, legend2,'Location','best');
        writeTitleAndLabels(horzcat([legend1, ' and ', ...
            legend2]), labelX, labelY);
    else
        plot(vectors(:, 1), vectors(:, 2), vectors(:, 1), ...
            vectors(:, 3), 'r--', vectors(:, 1), vectors(:, 4), 'g--');
    
        addExtras([vectors(:, 1), vectors(:, 1), vectors(:, 1), ...
            vectors(:, 2), vectors(:, 3), vectors(:, 4)], legend1, ...
            legend2, legend3, labelX, labelY) 
    end
    
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
