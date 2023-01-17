function out = fuzzy_log(l, lf, f, rf, r, target_angle)
    fis = readfis('obhod_youbot');
    out = evalfis(fis, [l, lf, rf, r, target_angle, f]);
end