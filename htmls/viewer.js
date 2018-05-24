

function calcStatus(){
	var str = Number($('#statusform [name=str]').val());
	var con = Number($('#statusform [name=con]').val());
	var pow = Number($('#statusform [name=pow]').val());
	var dex = Number($('#statusform [name=dex]').val());
	var app = Number($('#statusform [name=app]').val());
	var siz = Number($('#statusform [name=siz]').val());
	var int = Number($('#statusform [name=int]').val());
	var edu = Number($('#statusform [name=edu]').val());
	var statusArr1 = [str, con, pow, dex, app, siz, int, edu];
	console.log(statusArr1);
	var statusSum = str+con+pow+dex+app+siz+int+edu;
	$("#status-sum").html(statusSum);
	var san  = pow * 5;
	var luck = pow * 5;
	var idea = int * 5;
	var know = edu * 5; if (know >= 100) {know = 99;}
	var hp   = Math.ceil((con + siz) / 2);
	var mp   = pow;
	var db = "";
	var dbc = str + siz;
	if (dbc<=12){db="-1d6";}
	else if (dbc<=16){db="-1d4";}
	else if (dbc<=24){db="+0";}
	else if (dbc<=32){db="+1d4";}
	else {db="+1d6";}
	var statusArr2 =[hp, mp, san, idea, luck, know, db];
	console.log(statusArr2)

	var wak = "";
	for (i = 0; i < statusArr2.length; i++) {
		wak += "<td>";
		wak += statusArr2[i];
		wak += "</td>\n";
	}
	console.log(wak);
	$("#table2").html(wak);
}

$(function() {
	calcStatus();
});