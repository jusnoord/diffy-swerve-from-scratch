#!/usr/bin/env fish
# deploys code to defined device through rsync, and then runs the simulation with hwSim flag
# keep in mind that this requires ssh keys to be set up for passwordless login

set URL sparky@10.93.12.2 #todo: fix
set DESTINATION /home/sparky/diffy-swerve-from-scratch/

echo "Deploying to $URL:$DESTINATION"
rsync -avz --exclude-from='.rsync-exclude' ./ $URL:$DESTINATION
echo "Deployment complete!"

echo "running remote sim and piping output to terminal..."
ssh -t $URL "cd $DESTINATION && ./simulate.sh"