function val= sat(inp,phi)

if(abs(inp)>phi)
    val = sign(inp);
else
    val = inp/phi;
end
end