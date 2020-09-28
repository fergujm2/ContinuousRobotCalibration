function y = MakeExtendedKnots(a, b, k, d)
    numRep = d + 1;

    y0 = a*ones(1, numRep);
    yf = b*ones(1, numRep);

    yInt = linspace(a, b, k + 2);
    yInt = yInt(2:(end - 1));

    y = [y0, yInt, yf];
end