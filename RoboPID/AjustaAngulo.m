function [AjustaAngulo] = AjustaAngulo(angulo)

angulo = mod(angulo,360);
if angulo>180
    AjustaAngulo=angulo-360;
else
    AjustaAngulo=angulo;
end
end