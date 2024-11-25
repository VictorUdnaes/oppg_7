/* Navi

 Skjelettprogram for veinavigasjon.
 Ferdig GUI for å legge inn nodenumre og vise kartfliser (tiles) fra Openstreetmap.

 Det som mangler:
 * innlesing av graf fra fil
 * Dijkstras algoritme og ALT
	* plotte punkter på det grafiske kartet
   - nodenes bredde- og lengdegrader kan brukes for plotting på kartet

  Kompilere:
 javac Navi.java -cp JMapViewer.jar

 Kjøre programmet:
 java -cp .:JMapViewer.jar Navi

 Eller sett opp CLASSPATH så java finner JMapViewer.jar uten hjelp.

 */

import java.util.Date;

//GUI
import javax.swing.*; //Vinduer
import javax.swing.event.*;
import java.awt.*;
import java.util.List;
import java.awt.event.*;


//JMapViewer

import org.openstreetmap.gui.jmapviewer.events.JMVCommandEvent;
import org.openstreetmap.gui.jmapviewer.interfaces.JMapViewerEventListener;
import org.openstreetmap.gui.jmapviewer.interfaces.TileLoader;
import org.openstreetmap.gui.jmapviewer.interfaces.TileSource;
import org.openstreetmap.gui.jmapviewer.tilesources.BingAerialTileSource;
import org.openstreetmap.gui.jmapviewer.tilesources.OsmTileSource;

import org.openstreetmap.gui.jmapviewer.*;

//GUI
class vindu extends JPanel implements ActionListener, DocumentListener, JMapViewerEventListener {
	// Set global to and from nodes
	private static int fromNode = 2486870;
	private static int toNode = 5394165;
	JButton btn_dijkstra = new JButton("Dijkstra");
	JButton btn_alt = new JButton("ALT");
	JButton btn_slutt = new JButton("Avslutt");
	JLabel lbl_fra = new JLabel();
	JLabel lbl_til = new JLabel();
	JTextField txt_fra = new JTextField(30);
	JTextField txt_til = new JTextField(30);
	JLabel lbl_tur = new JLabel("-");
	JLabel lbl_alg = new JLabel("-");
	String gml_fra = "";
	String gml_til = "";
	JPanel kart = new JPanel(new BorderLayout());
	//JMapViewer stuff
	private final JMapViewerTree treeMap;
	private final JLabel zoomLabel;
	private final JLabel zoomValue;

	private final JLabel mperpLabelName;
	private final JLabel mperpLabelValue;

	ALT alt;

	Layer rutelag, areallag;

	public vindu(ALT alt, List<Node> nodes) {
		super(new GridBagLayout());
		this.alt = alt;
		GridBagConstraints c = new GridBagConstraints();
		GridBagConstraints hc =  new GridBagConstraints(); //høyrejustert
		GridBagConstraints vc =  new GridBagConstraints(); //venstrejustert

		btn_dijkstra.setActionCommand("dijkstra");
		btn_dijkstra.setMnemonic(KeyEvent.VK_D);
		btn_alt.setActionCommand("alt");

		btn_dijkstra.addActionListener(this);
		btn_alt.addActionListener(this);
		btn_slutt.addActionListener(this);

		txt_fra.getDocument().addDocumentListener(this);
		txt_til.getDocument().addDocumentListener(this);

		hc.gridx = 0; hc.gridy = 1;

		hc.anchor = GridBagConstraints.NORTHEAST;
		vc.anchor = GridBagConstraints.NORTHWEST;
		hc.fill = vc.fill = GridBagConstraints.NONE;

		add(new JLabel("Fra:"), hc);

		c.gridx = 1; c.gridy = 1;
		add(txt_fra, c);

		hc.gridx = 3;
		add(new JLabel("Til:"), hc);

		c.gridx = 4;
		add(txt_til, c);


		hc.gridx = 0; hc.gridy = 2;
		add(new JLabel("Node:"), hc);

		vc.gridx = 1; vc.gridy = 2;
		add(lbl_fra, vc);

		hc.gridx = 3;
		add(new JLabel("Node:"), hc);

		vc.gridx = 4;
		add(lbl_til, vc);


		c.gridx = 0; c.gridy = 3;
		add(btn_dijkstra, c);

		c.gridx = 1;
		add(btn_alt, c);

		vc.gridx = 1; vc.gridy = 4;
		vc.gridwidth = 3;
		add(lbl_tur, vc);

		vc.gridy = 5;
		add(lbl_alg, vc);

		c.gridx = 5; c.gridy = 6;
		add(btn_slutt, c);

		c.gridx = 0; c.gridy = 7;
		c.gridwidth = 6;
		c.gridheight = 5;
		c.fill = GridBagConstraints.BOTH;
		c.weightx = 1.0;
		c.weighty = 1.0;
		add(kart, c);

		treeMap = new JMapViewerTree("Lag");
		rutelag = treeMap.addLayer("kjørerute");
		areallag = treeMap.addLayer("undersøkt areal");
		map().addJMVListener(this);

		JPanel panel = new JPanel(new BorderLayout());
		JPanel panelTop = new JPanel();
		JPanel panelBottom = new JPanel();
		JPanel helpPanel = new JPanel();

		mperpLabelName = new JLabel("meter/Pixel: ");
		mperpLabelValue = new JLabel(String.format("%s", map().getMeterPerPixel()));

		zoomLabel = new JLabel("Zoomnivå: ");
		zoomValue = new JLabel(String.format("%s", map().getZoom()));

		kart.add(panel, BorderLayout.NORTH);
		kart.add(helpPanel, BorderLayout.SOUTH);
		panel.add(panelTop, BorderLayout.NORTH);
		panel.add(panelBottom, BorderLayout.SOUTH);
		JLabel helpLabel = new JLabel("Flytt med høyre musknapp,\n "
				+ "zoom med venstre eller dobbeltklikk.");
		helpPanel.add(helpLabel);
		JButton button = new JButton("setDisplayToFitMapMarkers");
		button.addActionListener(e -> map().setDisplayToFitMapMarkers());
		JComboBox<TileSource> tileSourceSelector = new JComboBox<>(new TileSource[] {
				new OsmTileSource.Mapnik(),
				new OsmTileSource.TransportMap(),
				new BingAerialTileSource(),
		});
		tileSourceSelector.addItemListener(new ItemListener() {
			@Override
			public void itemStateChanged(ItemEvent e) {
				map().setTileSource((TileSource) e.getItem());
			}
		});
		JComboBox<TileLoader> tileLoaderSelector;
		tileLoaderSelector = new JComboBox<>(new TileLoader[] {new OsmTileLoader(map())});
		tileLoaderSelector.addItemListener(new ItemListener() {
			@Override
			public void itemStateChanged(ItemEvent e) {
				map().setTileLoader((TileLoader) e.getItem());
			}
		});
		map().setTileLoader((TileLoader) tileLoaderSelector.getSelectedItem());
		panelTop.add(tileSourceSelector);
		panelTop.add(tileLoaderSelector);
		final JCheckBox showMapMarker = new JCheckBox("Map markers visible");
		showMapMarker.setSelected(map().getMapMarkersVisible());
		showMapMarker.addActionListener(e -> map().setMapMarkerVisible(showMapMarker.isSelected()));
		panelBottom.add(showMapMarker);
		///
		final JCheckBox showTreeLayers = new JCheckBox("Tree Layers visible");
		showTreeLayers.addActionListener(e -> treeMap.setTreeVisible(showTreeLayers.isSelected()));
		panelBottom.add(showTreeLayers);
		///
		final JCheckBox showToolTip = new JCheckBox("ToolTip visible");
		showToolTip.addActionListener(e -> map().setToolTipText(null));
		panelBottom.add(showToolTip);
		///
		final JCheckBox showTileGrid = new JCheckBox("Tile grid visible");
		showTileGrid.setSelected(map().isTileGridVisible());
		showTileGrid.addActionListener(e -> map().setTileGridVisible(showTileGrid.isSelected()));
		panelBottom.add(showTileGrid);
		final JCheckBox showZoomControls = new JCheckBox("Show zoom controls");
		showZoomControls.setSelected(map().getZoomControlsVisible());
		showZoomControls.addActionListener(e -> map().setZoomControlsVisible(showZoomControls.isSelected()));
		panelBottom.add(showZoomControls);
		final JCheckBox scrollWrapEnabled = new JCheckBox("Scrollwrap enabled");
		scrollWrapEnabled.addActionListener(e -> map().setScrollWrapEnabled(scrollWrapEnabled.isSelected()));
		panelBottom.add(scrollWrapEnabled);
		panelBottom.add(button);

		panelTop.add(zoomLabel);
		panelTop.add(zoomValue);
		panelTop.add(mperpLabelName);
		panelTop.add(mperpLabelValue);

		kart.add(treeMap, BorderLayout.CENTER);

		map().addMouseListener(new MouseAdapter() {
			@Override
			public void mouseClicked(MouseEvent e) {
				if (e.getButton() == MouseEvent.BUTTON1) {
					map().getAttribution().handleAttribution(e.getPoint(), true);
				}
			}
		});

		map().addMouseMotionListener(new MouseAdapter() {
			@Override
			public void mouseMoved(MouseEvent e) {
				Point p = e.getPoint();
				boolean cursorHand = map().getAttribution().handleAttributionCursor(p);
				if (cursorHand) {
					map().setCursor(new Cursor(Cursor.HAND_CURSOR));
				} else {
					map().setCursor(new Cursor(Cursor.DEFAULT_CURSOR));
				}
				if (showToolTip.isSelected()) map().setToolTipText(map().getPosition(p).toString());
			}
		});

	//------------------------------------------------------------------------------------------------------------------
	/*
	 * for å tegne ALT kjør tegn_alt();
	 * for å tegne Dijkstra kjør tegn_dijkstra();
	 * for å tegne POI kjør tegn_poi(nodes);
	 */
		//tegn_poi(nodes);
		tegn_alt();
		tegn_dijkstra();

	} //konstruktør for vindu

	public double grad(double rad) {
		return rad / Math.PI * 180;
	}

	public void tegn_poi(List<Node> POIS) {
		var graph = alt.getGraph();

		// Iterate over all POIS
		for (Node poi : POIS) {
			// Retrieve the node from the graph using its ID
			var node = graph.getNode(poi.id);
			if (node != null) {
				// Create a map marker for the node
				var marker = new MapMarkerDot(rutelag, node.latitude, node.longitude);

				// Set the marker color
				marker.setBackColor(Color.RED);

				// Add the marker to the map
				map().addMapMarker(marker);
			}
		}
	}

	public void tegn_dijkstra() {
		var graph = alt.getGraph();
		var node1 = graph.getNode(fromNode);
		var node2 = graph.getNode(toNode);
		var start = new MapMarkerDot(node1.latitude, node1.longitude);
		var end = new MapMarkerDot(node2.latitude, node2.longitude);
		start.setBackColor(Color.RED);
		end.setBackColor(Color.RED);
		map().addMapMarker(start);
		map().addMapMarker(end);
		//alt.setupPOIDistances(node1);
		long startTime = System.nanoTime();
		var path = alt.runDijkstrasAlgorithm(node1, node2);
		long endTime = System.nanoTime();
		System.out.println("Dijkstra time: " + (endTime - startTime)/1000000 + " ms");
		System.out.println("Dijkstra path size: " + path.size());
		long length = 0;
		long lastLength = 0;
		for (var node : path) {
			length += node.distanceFromStart - lastLength;
			lastLength = node.distanceFromStart;
		}
		long totalSeconds = length / 100;
		System.out.println("Total seconds: " + totalSeconds);
		// Finn timer, minutter og sekunder
		int hours = (int) (totalSeconds / 3600);
		int minutes = (int) ((totalSeconds % 3600) / 60);
		int seconds = (int) (totalSeconds % 60);
		System.out.println("Dijkstra path length: " + hours + " hours, " + minutes + " minutes, " + seconds + " seconds");
		Layer rutelag = treeMap.addLayer("Kjørerute");
		for (var node : path) {
			var marker = new MapMarkerDot(rutelag, node.latitude, node.longitude);
			marker.setBackColor(Color.BLUE);
			map().addMapMarker(marker);
		}
	}

	public void tegn_alt() {
		var graph = alt.getGraph();
		var node1 = graph.getNode(fromNode);
		var node2 = graph.getNode(toNode);
		var start = new MapMarkerDot(node1.latitude, node1.longitude);
		var end = new MapMarkerDot(node2.latitude, node2.longitude);
		start.setBackColor(Color.RED);
		end.setBackColor(Color.RED);
		map().addMapMarker(start);
		map().addMapMarker(end);
		alt.setupPOIDistances();
		long startTime = System.nanoTime();
		var path = alt.runAlt(node1, node2).path;
		long endTime = System.nanoTime();
		System.out.println("ALT time: " + (endTime - startTime)/1000000 + " ms");
		System.out.println("ALT path size: " + path.size());
		long length = 0;
		long lastLength = 0;
		for (var node : path) {
			length += node.distanceFromStart - lastLength;
			lastLength = node.distanceFromStart;
		}
		long totalSeconds = length / 100;

		// Finn timer, minutter og sekunder
		int hours = (int) (totalSeconds / 3600);
		int minutes = (int) ((totalSeconds % 3600) / 60);
		int seconds = (int) (totalSeconds % 60);
		System.out.println("Alt path length: " + hours + " hours, " + minutes + " minutes, " + seconds + " seconds");


		Layer rutelag = treeMap.addLayer("Kjørerute");
		for (var node : path) {
			var marker = new MapMarkerDot(rutelag, node.latitude, node.longitude);
			marker.setBackColor(Color.YELLOW);
			map().addMapMarker(marker);
		}
	}

	//Knapper
	public void actionPerformed(ActionEvent e) {
		int noder = 0;
		Date tid1 = new Date();
		String tur = "Kjøretur " + txt_fra.getText() + " - " + txt_til.getText();
		String alg = "";
		switch (e.getActionCommand()) {
			case "dijkstra":
				tegn_dijkstra();
				alg = "Dijkstras algoritme ";
				break;
			case "alt":
				/* sett inn kall for å kjøre ALT her */
				alg = "ALT-algoritmen ";
				break;
			default:
				System.exit(0);
				break;
		}
		Date tid2 = new Date();
		map().removeAllMapMarkers();

	}

	//Skriving i tekstfelt
	public void changedUpdate(DocumentEvent ev) {
	}
	public void removeUpdate(DocumentEvent ev) {
		stedsoppslag();
	}
	public void insertUpdate(DocumentEvent ev) {
		stedsoppslag();
	}

	//Finn hvilket felt som ble endret.
	//Slå opp nodenumre om mulig/ønskelig
	void stedsoppslag() {

	}

	//Noe skjer med kartet.
	public void processCommand(JMVCommandEvent command) {
		if (command.getCommand().equals(JMVCommandEvent.COMMAND.ZOOM) ||
				command.getCommand().equals(JMVCommandEvent.COMMAND.MOVE)) {
			updateZoomParameters();
		}
	}

	private void updateZoomParameters() {
		if (mperpLabelValue != null)
			mperpLabelValue.setText(String.format("%s", map().getMeterPerPixel()));
		if (zoomValue != null)
			zoomValue.setText(String.format("%s", map().getZoom()));
	}

	private JMapViewer map() {
		return treeMap.getViewer();
	}

}


//Java Navi
public class Navi {

	public static void gui(ALT alt, List<Node> nodes) {

		JFrame frame = new JFrame("Kartnavigasjon");

		//Innhold
		frame.add(new vindu(alt, nodes));

		//Vis vinduet
		frame.pack();
		frame.setVisible(true);
	}

	public static void main(String[] args) {
		ALT alt = new ALT();

		int startNodeId = 790843;
		int desiredTypeCode = 8;
		int numberOfTypes = 4;

		List<Node> foundPOIS = alt.findPOIsWithDijkstra(startNodeId, desiredTypeCode, numberOfTypes);

		System.out.println("Number of POIs found: " + foundPOIS.size());

		for (Node node : foundPOIS) {
			System.out.println("ID found for POI: " + node.id);

			// Retrieve the PoiType using the getter method
			ALT.PoiType poiType = alt.getPoiType(node.id);

			if (poiType != null) {
				System.out.println("Name of establishment: " + poiType.name);
			} else {
				System.out.println("No POI type found for node ID: " + node.id);
			}
		}
		gui(alt, foundPOIS);

	}
}
