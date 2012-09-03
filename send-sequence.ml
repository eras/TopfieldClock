open Batteries

let open_serial tty bps =
  let fd = Unix.openfile tty [Unix.O_RDWR; Unix.O_NONBLOCK] 0 in
  let tio = Unix.tcgetattr fd in
  let tio = {
    tio with
      Unix.c_clocal	= true;
      c_obaud		= bps;
      c_ibaud		= bps;
      c_csize		= 8;
      c_cstopb		= 1;
      c_inlcr           = false;
      c_icrnl		= false;
      c_opost           = false;
      c_isig		= false;
      c_icanon		= false;
      c_echo		= false;
      c_vtime		= 1;
      c_vmin		= 1;
  } in
    Unix.tcsetattr fd Unix.TCSANOW tio;
    Unix.clear_nonblock fd;
    fd

let write_all fd str = 
  for c = 0 to String.length str - 1 do
    ignore (Unix.select [] [] [] 0.001);
    ignore (Unix.write fd str c 1)
  done

type op = 
  | Bool of bool
  | DelayHigh

let binstr_of_bool = function
  | Bool false -> "0"
  | Bool true -> "1"
  | DelayHigh -> assert false

let bool_of_binstr = function 
  | '0' -> Some (Bool false)
  | '1' -> Some (Bool true)
  | '2' -> Some DelayHigh
  | _ -> None

let not1 f x = not (f x)

let split_by p xs =
  (List.take_while (not1 p) xs, 
   match List.drop_while (not1 p) xs with
     | [] -> []
     | _::xs -> xs)

let rec parts_by p xs =
  match xs with
    | [] -> []
    | _ -> 
	let (left, right) = split_by p xs in
	  left::parts_by p right

let send_sequence fd seq = 
  let seq = List.of_enum seq in
  let parts = parts_by ((=) DelayHigh) seq in
    List.iter
      (fun part ->
	 write_all fd "b";
	 write_all fd (String.concat "" (List.map binstr_of_bool part));
	 write_all fd ";";
	 ignore (Core.Core_unix.nanosleep 0.30000);
      )
      parts

let main () =
  let args = List.tl (Array.to_list Sys.argv) in
    match args with
      | serial_port::sequence::[] ->
	  let fd = open_serial serial_port 9600 in
	    send_sequence fd (Enum.filter_map bool_of_binstr (String.enum sequence));
	    Unix.close fd
      | _ -> 
	  Printf.printf "usage: send-sequence /dev/ttyUSB1 001010100012\n";
	  Printf.printf "Unsupported characters are ignored in the sequence\n"

let _ = main ()
