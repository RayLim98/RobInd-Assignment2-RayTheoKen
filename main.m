clc
clear
clf

bot = Sawyer(0,0,0);
bot.model.teach();
hold on

q0 = bot.model.getpos();
qr = [0 -pi -pi/2 pi/2 0 -pi/2 0];

qM = jtraj(q0,qr,30);

bot.model.plot(qM);
bot.model.teach();
