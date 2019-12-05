if (sim_call_type==sim.syscb_init) then
    phandle = sim.getObjectHandle("Pioneer_p3dx#0")
    pos_orient_pub = simROS.advertise("/pos_orient_","geometry_msgs/Vector3")
    simROS.publisherTreatUInt8ArrayAsString(pos_orient_pub)
    
    vel_lin_ang_pub = simROS.advertise("/vel_lin_ang", "geometry_msgs/Vector3")
    simROS.publisherTreatUInt8ArrayAsString(vel_lin_ang_pub)
end

function rad2deg(rad_)
    return 180.0/math.pi * rad_
end

if (sim_call_type==sim.syscb_sensing) then
    posisi = sim.getObjectPosition(phandle, -1)
    pos_ = {x=posisi[1], y=posisi[2]}
    orientasi = sim.getObjectOrientation(phandle, -1)
    pos_["z"] = rad2deg(orientasi[3])
    simROS.publish(pos_orient_pub, pos_)

    lin_vel_val, ang_vel_val = sim.getObjectVelocity(phandle)
    vel_lin_ang_val = {}
    vel_lin_ang_val["x"] = lin_vel_val[1]
    vel_lin_ang_val["y"] = lin_vel_val[2]
    vel_lin_ang_val["z"] = ang_vel_val[3]
    simROS.publish(vel_lin_ang_pub, vel_lin_ang_val)
end
