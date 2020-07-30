function AB = GetRandomAB(n, numJoints)
    mag = 0.01;
    A = [[0, 0, pi, -pi/3, pi, 0];
        mag*(2.*rand(n, numJoints) - 1)];
    B = mag*(2.*rand(n, numJoints) - 1);
    AB = [A; B];
end