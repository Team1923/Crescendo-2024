package frc.lib.AutoUtils;


import com.pathplanner.lib.commands.PathPlannerAuto;

public class AutoInstantiator {

	public static AutoInstantiator autoInstantiator;

	
	public static synchronized AutoInstantiator getInstance() {
		if (autoInstantiator == null) {
			autoInstantiator = new AutoInstantiator();
		}
		return autoInstantiator;
	}

	public AutoInstantiator() {

	}



}