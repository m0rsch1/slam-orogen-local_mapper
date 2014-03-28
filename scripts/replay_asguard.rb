#! /usr/bin/env ruby
#library for displaying data
require 'vizkit'
require 'readline'
require 'eigen'
require 'rock/bundle'

if !ARGV[0]  then 
    puts "usage: replay.rb log_dir"
    exit
end


#load log file 
log = Orocos::Log::Replay.open(ARGV[0])
Orocos::CORBA::max_message_size = 100000000

log.track(true)
#log.traversability.track(false) 
log.transformer_broadcaster.track(false) 
log.transformer_broadcaster.rename('foo')
log.name_service.deregister 'local_mapper'
#log.name_service.deregister 'transformer_broadcaster'

Bundles.initialize
# Use rock-bundle-default spacebot before
Bundles.transformer.load_conf(Bundles.find_file('config', 'transforms_asguardv3.rb'))

Bundles.run 'local_mapper::Task' => 'local_mapper', :valgrind => false, :output => nil do |p|
    
    odometry = Orocos.name_service.get 'odometry'
    hokuyo_front = Orocos::TaskContext.get('laser_filter_front')
    local_mapper = Bundles::get 'local_mapper'

    hokuyo_front.filtered_scans.connect_to(local_mapper.scan_samples, :type => :buffer, :size => 100)
    #odometry.odometry_samples.connect_to(local_planner.odometry_samples, :type => :buffer, :size => 100)
    
    local_mapper.apply_conf(['default'])
    
    
    Bundles.transformer.setup( local_mapper )
    local_mapper.configure()
    local_mapper.start()

    Vizkit.control log

    
    Vizkit.display odometry.odometry_samples, :widget => Vizkit.default_loader.RigidBodyStateVisualization
    Vizkit.display local_mapper.map

    Vizkit.exec()
end

