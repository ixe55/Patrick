# MeshCat animation for the 2-DOF ball-on-plate
# X[k] must start with [px, py, α, β, ...]
#   α tilts the plate about its y-axis (+α raises the +x edge → ball rolls toward -x)
#   β tilts the plate about its x-axis (+β raises the +y edge → ball rolls toward -y)
# Optional servo linkage drawing: pass θs (vector of [θx, θy] horn angles) and linkage geometry.

function plate_rotation(α, β)
    Rx = [1.0 0 0; 0 cos(β) -sin(β); 0 sin(β) cos(β)]
    Ry = [cos(α) 0 -sin(α); 0 1.0 0; sin(α) 0 cos(α)]   # = RotY(-α)
    return Rx * Ry
end

# transform that maps a unit-length box along z (centered at origin) onto segment a → b
function segment_transform(a, b)
    v = b - a
    ℓ = norm(v)
    e3 = v / ℓ
    tmp = abs(e3[1]) < 0.9 ? [1.0, 0, 0] : [0.0, 1, 0]
    e1 = normalize(cross(tmp, e3))
    e2 = cross(e3, e1)
    Rm = [e1 e2 e3] * Diagonal([1.0, 1.0, ℓ])
    return mc.Translation((a + b)/2) ∘ mc.LinearMap(Rm)
end

function animate_ball_plate(X, dt; L = 0.30, t_plate = 0.01, r = 0.02,
                            Xref = nothing, n_ref_markers = 60,
                            θs = nothing, d_attach = 0.08, r_horn = 0.02, L_rod = 0.10)
    vis = mc.Visualizer()
    gray(c) = mc.MeshPhongMaterial(color = mc.RGBA(c, c, c, 1.0))
    seg(w) = mc.HyperRectangle(mc.Vec(-w/2, -w/2, -0.5), mc.Vec(w, w, 1.0))

    mc.setobject!(vis[:plate],
        mc.HyperRectangle(mc.Vec(-L/2, -L/2, -t_plate/2), mc.Vec(L, L, t_plate)),
        mc.MeshPhongMaterial(color = mc.RGBA(0.25, 0.45, 0.85, 1.0)))
    mc.setobject!(vis[:pedestal],
        mc.HyperRectangle(mc.Vec(-0.015, -0.015, -0.30), mc.Vec(0.03, 0.03, 0.28)), gray(0.3))
    mc.setobject!(vis[:pivot], mc.HyperSphere(mc.Point(0.0, 0.0, -0.02), 0.02), gray(0.2))
    mc.setobject!(vis[:ball], mc.HyperSphere(mc.Point(0.0, 0.0, 0.0), r),
        mc.MeshPhongMaterial(color = mc.RGBA(1.0, 0.5, 0.0, 1.0)))

    if Xref !== nothing
        idx = round.(Int, range(1, length(Xref), length = n_ref_markers))
        for (j, i) in enumerate(idx)
            mc.setobject!(vis[:plate][:ref][Symbol("p$j")],
                mc.HyperSphere(mc.Point(Xref[i][1], Xref[i][2], t_plate/2), 0.003),
                mc.MeshPhongMaterial(color = mc.RGBA(0.9, 0.1, 0.1, 1.0)))
        end
    end

    # servo geometry (schematic: rod length is not held exactly constant)
    z0 = -t_plate/2 - L_rod
    piv = [[d_attach - r_horn, 0.0, z0], [0.0, d_attach - r_horn, z0]]   # horn pivots
    hdir(i, θ) = i == 1 ? [cos(θ), 0.0, sin(θ)] : [0.0, cos(θ), sin(θ)]
    att = [[d_attach, 0.0, -t_plate/2], [0.0, d_attach, -t_plate/2]]      # plate frame
    if θs !== nothing
        for i = 1:2
            c = piv[i] - [0, 0, 0.02]
            mc.setobject!(vis[Symbol("servo$i")],
                mc.HyperRectangle(mc.Vec((c .- [0.012, 0.012, 0.02])...), mc.Vec(0.024, 0.024, 0.03)), gray(0.1))
            mc.setobject!(vis[Symbol("horn$i")], seg(0.006), mc.MeshPhongMaterial(color = mc.RGBA(0.9, 0.9, 0.2, 1.0)))
            mc.setobject!(vis[Symbol("rod$i")], seg(0.004), gray(0.75))
        end
    end

    fps = floor(Int, 1/dt)
    anim = try
        mc.Animation(vis; fps = fps)
    catch
        mc.Animation(fps)
    end

    for k = 1:length(X)
        mc.atframe(anim, k) do
            Rp = plate_rotation(X[k][3], X[k][4])
            mc.settransform!(vis[:plate], mc.LinearMap(Rp))
            mc.settransform!(vis[:ball], mc.Translation(Rp * [X[k][1], X[k][2], t_plate/2 + r]))
            if θs !== nothing
                for i = 1:2
                    tip = piv[i] + r_horn*hdir(i, θs[k][i])
                    mc.settransform!(vis[Symbol("horn$i")], segment_transform(piv[i], tip))
                    mc.settransform!(vis[Symbol("rod$i")], segment_transform(tip, Rp*att[i]))
                end
            end
        end
    end

    mc.setanimation!(vis, anim)
    return mc.render(vis)
end
