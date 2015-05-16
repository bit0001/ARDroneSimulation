%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function setAxis(x1, x2, x3, y1, y2, y3)
    minX = min(min(min(x1), min(x2)), min(x3));
    maxX = max(max(max(x1), max(x2)), max(x3));
    minY = min(min(min(y1), min(y2)), min(y3));
    maxY = max(max(max(y1), max(y2)), max(y3));
    axis([minX maxX minY maxY])
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
