clear;

d = 2;
k = 4;

n = k + d + 1; % Dimension of the spline space

a = 0;
b = 1;

y(1:d) = a*ones(1, d);
y((n + 2):(n + d + 1)) = b*ones(1, d);
y((d + 1):(n + 1)) = linspace(a, b, k + 2);

C = eye(n);
t = linspace(0, 1, 10000);

h = figure(1);
clf;
h.Color = [1,1,1];
hold on;

rgbTable = 1./255.*[237,  32,  36;
                    247, 148,  32;
                    247, 236,  19;
                    106, 189,  69;
                     57,  83, 164;
                    102,  47, 144;
                    255,   0, 255];
            
for iii = 1:n
    xi = sval2(d, y, C(iii,:), t);
    plot(t, xi, 'Color', rgbTable(iii,:), 'LineWidth', 0.5);
end

c = [1.4, 1.9, 1, 0.9, 1.7, 1.3, 1.9];
x = sval2(d, y, c, t);
plot(t, x, '-k', 'LineWidth', 0.75);
    
xlabel('$t$', 'interpreter', 'latex');
ylabel('$f(t)$', 'interpreter', 'latex');

ax = gca;
ax.FontSize = 8;
    
saveFigurePdf([3.5, 1.5]);


function saveFigurePdf(sz)
    h = gcf;
    set(gcf, 'PaperPosition', [0, 0, sz]);
    set(gcf, 'PaperSize', sz);
    saveas(gcf, fullfile('Figures', ['Figure', num2str(h.Number)]), 'pdf');
end