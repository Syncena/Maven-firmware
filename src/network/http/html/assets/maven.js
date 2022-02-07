var ping_sequence_target = 99999;
var ping_sequence_netsum = 99999;
var ping_sequence_network = 99999;
var ping_sequence_comms = 99999;
var ping_sequence_debug = 99999;
var wifi_pass = "";

function maven_details()
{
	$.getJSON({url:"maven.json", cache:false})
		.done(function (resp) {
			$(".maven-fwver").html(resp.fwver);
			$(".maven-fwdate").html(resp.fwdate);
			$(".maven-serial").html(resp.serial);
		});
}

function check_ping()
{
	$.getJSON({url:"ping.json", cache:false})
		.done(function (resp) {
			if (resp.target != ping_sequence_target) {
				ping_sequence_target = resp.target;
				target_refresh();
			}
			if (resp.netsum != ping_sequence_netsum) {
				ping_sequence_netsum = resp.netsum;
				netsum_refresh();
			}
			if (resp.network != ping_sequence_network) {
				ping_sequence_network = resp.network;
				network_refresh();
			}
			if (resp.debug != ping_sequence_debug) {
				ping_sequence_debug = resp.debug;
				debug_refresh();
			}
			if (resp.comms != ping_sequence_comms) {
				ping_sequence_comms = resp.comms;
				comms_refresh();
			}
		})
		.fail(function (xhr, status, err) {
			if (ping_sequence_target != 99999) {
				ping_sequence_target = 99999;
				target_clear();
			}
			if (ping_sequence_netsum != 99999) {
				ping_sequence_netsum = 99999;
				netsum_clear();
			}
			if (ping_sequence_network != 99999) {
				ping_sequence_network = 99999;
				network_clear();
			}
			if (ping_sequence_debug != 99999) {
				ping_sequence_debug = 99999;
			}
			if (ping_sequence_comms != 99999) {
				ping_sequence_comms = 99999;
			}
		});
	setTimeout(check_ping, 5000);
}

function submit_form(form, url)
{
	var formdata;

	formdata = $(form).serialize();
	$.post(url, formdata)
		.done(function(resp) {
			console.log("Form posted: " + resp);
			if (resp.trim() != "OK") {
				alert(resp);
			}
		})
		.fail(function(xhr, st, err) {
			console.log("Form failed: " + st + ", err " + err);
			alert("Failed to submit form!");
		});

	return true;
}

function target_update_summary(v, s)
{
	$(".target-vendor").html(v);
	$(".target-soc").html(s);
	$(".target-name").html(v + "&nbsp;" + s);
}

function yesno(b)
{
	if (b == true)
		return "yes";
	else
		return "no";
}

function target_update(ti)
{
	var i, table;

	target_update_summary(ti.vendor, ti.soc);
	$(".target-vcc").html(ti.tvcc);
	$(".target-extra").html(ti.extra0 + " " + ti.extra1);

	table = "<tr>";
	table += "<td class=\"leftspacer\">";
	table += "<td class=\"rightspacer\">";

	for (let i of ti.cores) {
		table += "<tr>";
		if (ti.cores.length > 1)
			table += "<td class=\"subhead\" nowrap>CPU Core #" + i.num;
		else
			table += "<td class=\"subhead\" nowrap>CPU Core";
		table += "<td class=\"rc\" nowrap>&nbsp;";

		table += "<tr>";
		table += "<td class=\"lc\" nowrap>Family";
		table += "<td class=\"rc\" nowrap>" + i.name;

		table += "<tr>";
		table += "<td class=\"lc\" nowrap>Architecture";
		table += "<td class=\"rc\" nowrap>ARMv" + i.vers + "-M " + i.imp + " implementation";

		table += "<tr>";
		table += "<td class=\"lc\" nowrap>Revision";
		table += "<td class=\"rc\" nowrap>" + i.rev;

		table += "<tr>";
		table += "<td class=\"lc\" nowrap>FPU";
		table += "<td class=\"rc\" nowrap>" + i.fpu;

		table += "<tr>";
		table += "<td class=\"lc\" nowrap>MPU";
		table += "<td class=\"rc\" nowrap>" + yesno(i.mpu);

		table += "<tr>";
		table += "<td class=\"lc\" nowrap>Cache";
		table += "<td class=\"rc\" nowrap>" + i.cache;

		table += "<tr>";
		table += "<td class=\"lc\" nowrap>HW Breakpoints";
		table += "<td class=\"rc\" nowrap>" + i.hwbreak;

		table += "<tr>";
		table += "<td class=\"lc\" nowrap>HW Watchpoints";
		table += "<td class=\"rc\" nowrap>" + i.hwwatch;

		if (i.core_notes.trim() != "") {
			table += "<tr>";
			table += "<td class=\"lc\" nowrap>Notes";
			table += "<td class=\"rc\">" + i.core_notes.trim();
		}

		if (i.memory.length > 0) {
			table += "<tr>";
			table += "<td class=\"subhead\" nowrap>Private Memories";
			table += "<td class=\"rc\" nowrap>&nbsp;";

			for (let m of i.memory) {
				table += format_memory(m);
			}
		}

		table += "<tr>";
		table += "<td class=\"leftspacer\">";
		table += "<td class=\"rightspacer\">";
	}

	$("#cpu-table").html(table);

	table = "<tr>";
	table += "<td class=\"subhead\" nowrap>";
	if (ti.cores.length > 1)
		table += "Shared ";
	table += "Memories";
	table += "<td class=\"rc\" nowrap>&nbsp;";

	for (let m of ti.memory) {
		table += format_memory(m);
	}

	$("#mem-table").html(table);
}

function format_memory(m)
{
	var rv;

	rv = "<tr>";
	rv += "<td class=\"lc-fixed\" nowrap>" + m.range;
	rv += "<td class=\"rc\" nowrap>" + m.size + " of " + m.type;

	if (m.desc.trim() != "") {
		rv += ", " + m.desc;
	}

	return rv;
}

function target_clear()
{
	target_update_summary("", "");
	$(".target-vcc").html("");
	$(".target-extra").html("");
	$("#cpu-table").html("");
	$("#mem-table").html("");
}

function target_refresh()
{
	$.getJSON({url:"target.json", cache:false})
		.done(function (resp) {
			target_update(resp);
		})
		.fail(function (xhr, status, err) {
			ping_sequence_target = 99999;
			target_clear();
		});
}

/*
 * Network id/name list. Form ID "network-form"
 *
 *   id/name net-wifi-ssid	WiFi SSID, text, 1-32 characters
 *   id/name net-wifi-security	WiFi security, 0=None, 1=WPA
 *   id/name net-wifi-pass	WiFi password, 8-64 characters, disable if net-wifi-security=0
 *   id/name net-ip-assignment	IP address, 0=Static, 1=DHCP.
 *   id/name net-ip-hostname	DHCP hostname, text, 1-64 chars, disable if net-ip-assignment=0
 *   id/name net-ip-addr	Static IP, text, 7-15 chars. disable if net-ip-assignment=1
 *   id/name net-ip-mask	Static IP, text, 7-15 chars. disable if net-ip-assignment=1
 *   id/name net-ip-gate	Static IP, text, 0,7-15 chars. disable if net-ip-assignment=1
 */
function network_dhcp_update(do_dhcp)
{
	/*
	 * Invoked in response to configuration updates from Maven,
	 * and also when the user clicks DHCP/Static radio buttons
	 * on the web page.
	 */
	$("#net-ip-addr").prop("disabled", do_dhcp);
	$("#net-ip-mask").prop("disabled", do_dhcp);
	$("#net-ip-gate").prop("disabled", do_dhcp);

	$("#net-ip-hostname").prop("disabled", !do_dhcp);
	if (do_dhcp == true) {
		$("#net-ip-hostname").val(net_ip_hostname);
	} else {
		$("#net-ip-hostname").val("");
	}
}

function network_wifi_sec_update(sec)
{
	if (sec != "0") {
		$("#net-wifi-pass").val(wifi_pass);
		$("#net-wifi-pass").prop("disabled", false);
	} else {
		$("#net-wifi-pass").val("");
		$("#net-wifi-pass").prop("disabled", true);
	}
}

function network_wifi_sec_change()
{
	/*
	 * Invoked when the user changes the WiFi Encryption Type
	 * opion on the web page.
	 */
	network_wifi_sec_update($("#net-wifi-security").val());
}

function network_update(net)
{
	if (net.dhcp == true) {
		$("#net-ip-dhcp").prop("checked", true);
		$("#net-ip-static").prop("checked", false);
	} else {
		$("#net-ip-dhcp").prop("checked", false);
		$("#net-ip-static").prop("checked", true);
	}

	$("#net-wifi-ssid").val(net.ssid);
	$("#net-wifi-security").val(net.wifi_sec);

	if (net.wifi_sec != "0") {
		wifi_pass = net.wifi_pass;
	} else {
		wifi_pass = "";
	}

	net_ip_hostname = net.hostname;
	$("#net-ip-addr").val(net.ip_addr);
	$("#net-ip-mask").val(net.ip_mask);
	$("#net-ip-gate").val(net.ip_gate);

	network_wifi_sec_update(net.wifi_sec);
	network_dhcp_update(net.dhcp);
}

function network_clear()
{
	$("#net-wifi-ssid").val("");
	$("#net-wifi-pass").val("");
	wifi_pass = "";
	network_wifi_sec_update("0");
	network_dhcp_update(true);
}

function network_refresh()
{
	$.getJSON({url:"network.json", cache:false})
		.done(function (resp) {
			network_update(resp);
		})
		.fail(function (xhr, status, err) {
			ping_sequence_network = 99999;
			network_clear();
		});
}

function netsum_update(ns)
{
	$(".netsum-ssid").html(ns.ssid);
	$(".netsum-chan").html(ns.chan);
	$(".netsum-ip").html(ns.addr);

	var sig;
	if (ns.sig == 4)
		sig = "Excellent";
	else if (ns.sig == 3)
		sig = "Good";
	else if (ns.sig == 2)
		sig = "Fair";
	else if (ns.sig == 1)
		sig = "Poor";
	else
		sig = "None";
	$(".netsum-sig").html(sig);
}

function netsum_clear()
{
	$(".netsum-ssid").html("");
	$(".netsum-chan").html("");
	$(".netsum-ip").html("");
	$(".netsum-sig").html("");
}

function netsum_refresh()
{
	$.getJSON({url:"netsum.json", cache:false})
		.done(function (resp) {
			netsum_update(resp);
		})
		.fail(function (xhr, status, err) {
			ping_sequence_netsum = 99999;
			netsum_clear();
		});
}

function debug_update(dbg)
{
	if (dbg.attach_reset == "reset") {
		$("#gdb-attach-reset").prop("checked", true);
		$("#gdb-attach-halt").prop("checked", false);
	} else {
		$("#gdb-attach-reset").prop("checked", false);
		$("#gdb-attach-halt").prop("checked", true);
	}

	if (dbg.attach_auto == "auto") {
		$("#gdb-attach-auto").prop("checked", true);
		$("#gdb-attach-manual").prop("checked", false);
	} else {
		$("#gdb-attach-auto").prop("checked", false);
		$("#gdb-attach-manual").prop("checked", true);
	}

	if (dbg.stepisr == "mask") {
		$("#gdb-step-mask").prop("checked", true);
		$("#gdb-step-unmask").prop("checked", false);
	} else {
		$("#gdb-step-mask").prop("checked", false);
		$("#gdb-step-unmask").prop("checked", true);
	}

	$("#gdb-vc-none").prop("selected", false);
	$("#gdb-vc-harderr").prop("selected", dbg.vc_hard);
	$("#gdb-vc-interr").prop("selected", dbg.vc_int);
	$("#gdb-vc-buserr").prop("selected", dbg.vc_bus);
	$("#gdb-vc-staterr").prop("selected", dbg.vc_stat);
	$("#gdb-vc-chkerr").prop("selected", dbg.vc_chk);
	$("#gdb-vc-nocperr").prop("selected", dbg.vc_nocp);
	$("#gdb-vc-mmerr").prop("selected", dbg.vc_mm);

	$("#swd-speed").val(dbg.swd_speed);
	$("#swd-speed-1").html(dbg.swd_speed_1);
	$("#swd-speed-2").html(dbg.swd_speed_2);
	$("#swd-speed-3").html(dbg.swd_speed_3);
	$("#swd-speed-4").html(dbg.swd_speed_4);
	$("#swd-speed-5").html(dbg.swd_speed_5);
	$("#swd-speed-6").html(dbg.swd_speed_6);
	$("#swd-speed-7").html(dbg.swd_speed_7);
	$("#swd-speed-8").html(dbg.swd_speed_8);
	$("#swd-speed-9").html(dbg.swd_speed_9);
	$("#swd-idlecycles").val(dbg.swd_idlecycles);
	$("#swd-turnaround").val(dbg.swd_turnaround);
}

function debug_refresh()
{
	$.getJSON({url:"debug.json", cache:false})
		.done(function (resp) {
			debug_update(resp);
		})
		.fail(function (xhr, status, err) {
			ping_sequence_debug = 99999;
		});
}

function comms_update(comms)
{
	$("#tc-serial-usb").val(comms.serial.usb);
	$("#tc-serial-tcp").val(comms.serial.tcp);
	$("#tc-serial-mode").val(comms.serial.mode);
	$("#tc-serial-baud").val(comms.serial.baud);
	$("#tc-serial-data-bits").val(comms.serial.bits);
	$("#tc-serial-parity").val(comms.serial.parity);
	$("#tc-serial-stop-bits").val(comms.serial.stop);

	$("#tc-swo-usb").val(comms.swo.usb);
	$("#tc-swo-tcp").val(comms.swo.tcp);
	$("#tc-swo-mode").val(comms.swo.mode);
	$("#tc-swo-baud").val(comms.swo.baud);

	$("#tc-semi-usb").val(comms.semihost0.usb);
	$("#tc-semi-tcp").val(comms.semihost0.tcp);
	$("#tc-semi-mode").val(comms.semihost0.mode);

	$("#tc-gdb-usb").val(comms.gdb0.usb);
	$("#tc-gdb-tcp").val(comms.gdb0.tcp);

	$("#tc-usb-vendor").val(comms.usb.vendor);
	$("#tc-usb-device").val(comms.usb.device);
}

function comms_refresh()
{
	$.getJSON({url:"targetcomms.json", cache:false})
		.done(function (resp) {
			comms_update(resp);
		})
		.fail(function (xhr, status, err) {
			ping_sequence_comms = 99999;
		});
}

function open_main(evt, contentName)
{
	var i, mc, sb;

	mc = document.getElementsByClassName("main-content");
	for (i = 0; i < mc.length; i++) {
		mc[i].style.display = "none";
	}

	sb = document.getElementsByClassName("nav-button");
	for (i = 0; i < sb.length; i++) {
		sb[i].className = sb[i].className.replace(" active", "");
	}

	document.getElementById(contentName).style.display = "block";
	evt.currentTarget.className += " active";
}
