%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function addExtras(vectors, legend1, legend2, legend3, labelX, labelY)
    legend(legend1, legend2, legend3,'Location','best');
    setAxis(vectors(:, 1), vectors(:, 2), vectors(:, 3), ...
        vectors(:, 4), vectors(:, 5), vectors(:, 6));
    writeTitleAndLabels(horzcat([legend1, ' ', legend2, ' and ', ...
        legend3]), labelX, labelY);
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
