# -*- coding: utf-8 -*-
"""
Created on Sun Nov 14 22:54:52 2021

@author: DiegoB
"""

import asyncio

from mavsdk import System
from mavsdk.offboard import (OffboardError, PositionNedYaw)

async def run():
    
    global drone_home_pos 
    global cur_goal_point_pos
    
    drone = System()
    await drone.connect(system_address = "udp://:14540") //Ambiente simulado
    
    print("Esperando conexión con dron")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print(f"Dron descubierto con UUID: {state.uuid}")
            break
        
        print("--Armando")
        await drone.action.arm()
        
        print("Esperando obtener la posición inicial")
        await get_drone_home_pos(drone)
        
        print("--Declarando posicion inicial")
        await drone.offboard.set_position_ned(PositionNedYaw(0.0,0.0,0.0,0.0))
        
        print("Iniciando modo offboard")
        try:
            await drone.offboard.start()
        except OffBoardError as error:
            print(f"Inicio de modo offboard falló con código: {error._result.result}")
            print("--Desarmando")
            await drone.action.disarm()
            return
        
    c_letter = [[0.0,0.0,-5.0,0.0],[1.0,-1.0,-5.0,30.0],[1.0,-3.0,-5.0,60.0],
                [0.0,-4.0,-5.0,90.0],[-3.0,-4.0,-5.0,120.0],[-4.0,-3.0,-5.0,180.0],
                [-4.0,-1.0,-5.0,210.0],[-3.0,0.0,-5.0,0.0]]
    
    for goal_point in c_letter:
        
        north_coord = goal_point[0]
        east_coord = goal_point[1]
        down_coord = goal_point[2]
        yaw_angle = goal_point[3]
        
        goal_point_angle = math.atan2(north_coord,east_coord)
        distance_to_goal_point = math.sqrt((east_coord**2)+(north_coord**2)) / 1000.0
        
        goal_point_bearing = (450-math.degrees(goal_point_angle)) % 360
        
        dest_latlong = get_dest_latlong(drone_home_pos['lat'],drone_home_pos['lon'],
                                        distance_to_goal_point,goal_point_bearing)
        
        cur_goal_point_pos['lat'] = dest_latlong[0]
        cur_goal_point_pos['lon'] = dest_latlong[1]
        cur_goal_point_pos['rel_alt'] = -down_coord
        
        await drone.offboard.set_position_ned(
            PositionNedYaw(north_coord,east_coord,down_coord,yaw_angle))
        
        await chek_is_at_goal(drone)
        

#Funciones
async def get_drone_home_pos(drone):
    
    global drone_home_pos
    
    async for home_position in drone.telemetry.home():
        
        drone_home_pos['lat'] = home_position.latitude_deg
        drone_home_pos['lon'] = home_position.longitude_deg
        drone_home_pos['rel_alt'] = home_position.relative_altitude_m
        
        return
    
async def check_is_at_goal(drone):
    global cur_goal_point_pos
    drone_current_pos = {'lat':None,'lon':None,'rel_lat':None}
    
    prev_round_hor_dist_to_goal = None
    
    async for position in drone.telemetry.position():
        
        drone_current_pos['lat'] = position.latitude_deg
        drone_current_pos['lon'] = position.longitude_deg
        drone_current_pos['lat'] = position.relative_altitude_m
        
        hor_dist_to_goal = 1000.0* get_haversine_distance(drone_current_pos['lat'],
                                                          drone_current_pos['lon'],
                                                          cur_goal_point_pos['lat'],
                                                          cur_goal_point_pos['lon'])
        
        ver_dist_to_goal = abs(cur_goal_point_pos['rel_alt'] - drone_current_pos['rel_alt'])
        
        round_hor_dist_to_goal = round(hor_dist_to_goal)
        
        if round_hor_dist_to_goal != prev_round_hor_dist_to_goal:
            prev_round_hor_dist_to_goal = round_hor_dist_to_goal
            print(f"...Distancia horizontal hasta punto objetivo: {round_hor_dist_to_goal}")
            
        if hor_dist_to_goal <= MAX_HOR_DIST_ERROR and ver_dist_to_goal <= MAX_VER_DIST_ERROR:
            print(f"Punto objetivo alcanzado!, hor_dist_to_goal = {(hor_dist_to_goal):.2f} mts")
            return
        
        
if __name__ == "__main__":
    loop = asyncio.get_event_loop()
    loop.run_until_complete(run())
        
            
    