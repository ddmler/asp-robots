<?php

$clingo_path = "/home/enrico/Schreibtisch/clingo/build/release/clingo"; // Full path

echo "\n";
echo "\n";
echo "\n";

$dir = scandir(__DIR__ . '/test');

foreach ($dir as $file) {
	if (substr($file, -3, 3) == ".lp") {
		$result_file = file_get_contents(__DIR__ . '/test/result/' . $file . '.txt');
		$results = explode(' ', $result_file);

		$clingo_all = shell_exec($clingo_path . ' rob.lp test/' . $file . ' 0');

		$clingo = explode('Answer:', $clingo_all);
		$clingo = array_pop($clingo);

		foreach ($results as $result) {
			if (stripos($clingo, $result) === false) {
				echo shell_exec($clingo_path . ' rob.lp test/' . $file . ' 0');
				echo "\n\n\n\033[31mFEHLER: Test " . $file . " ist fehlgeschlagen. \033[0mKonnte nicht finden:\n" . $result . "\n\nOben ist die Ausgabe von Clingo für diesen Test.\n";
				exit;
			}
		}
	}
}

echo "\033[32mEverything OK!\n";
