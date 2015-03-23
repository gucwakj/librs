# add each subdirectory
foreach (RSLIBS rs rsRobots rsXML rsScene rsSim rsCallback)
	add_subdirectory (${RSLIBS})
endforeach ()

