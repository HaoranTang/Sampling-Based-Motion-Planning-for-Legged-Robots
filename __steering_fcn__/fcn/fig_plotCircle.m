function fig_plotCircle(c,r)

th = linspace(0,2*pi,50);
th = [th,th(1)];

x = c(1) + r * cos(th);
y = c(2) + r * sin(th);

patch(x,y,'b')


