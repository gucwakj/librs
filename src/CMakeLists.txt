# add each subdirectory
foreach (RSLIBS rsRobots rsXML rsScene rsSim rsCallback)
	add_subdirectory (${RSLIBS})
endforeach ()

