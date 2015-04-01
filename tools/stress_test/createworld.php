#!/usr/bin/env php
<?xml version="1.0" ?>
<?php
$test = (int)$argv[1];
$nBots = (int)$argv[2];
$radius = (float)$argv[3];
$bot = "stress_".$test;
?>
<sdf version="1.5">
  <world name="default">
    <plugin name="stress" filename="libtolstress.so" />
    <physics type="ode">
      <!-- dt, typically -->
      <max_step_size>0.002</max_step_size>
      <!-- As fast as possible -->
      <real_time_update_rate>0</real_time_update_rate>     
      <!--<max_contacts>32</max_contacts>-->
      <ode>
        <constraints>
          <!-- Simulator.cpp -->
          <erp>0.1</erp>
          <cfm>10e-6</cfm>
        </constraints>
      </ode>
    </physics>
    <!-- A global light source -->
    <include>
      <uri>model://sun</uri>
    </include>
    <include>
      <uri>model://tol_ground</uri>
    </include>

<?php
$shift = 2.0 * M_PI / $nBots;
for ($i = 0; $i < $nBots; $i++) {
  $angle = $i * $shift;
  $x = $radius * cos($angle);
  $y = $radius * sin($angle);
  $yaw = M_PI + $angle;
?>
    <include>
      <uri>model://<?=$bot?></uri>
      <pose><?=$x?> <?=$y?> 0.02 0 0 <?=$yaw?></pose>
    </include>
<?php
}
?>

  </world>
</sdf>
