$(window).load(function(){
	var width = $(window).width();
	var height = $(window).height();
	
	$("#homeBox").css('left', ((width - $("#homeBox").width() ) / 2 )  + 'px');
	$("#aboutBox").css('left', ((width - $("#aboutBox").width() ) / 2 )  + 'px');
	$("#options").css('left', ((width - $("#options").width() ) / 2 )  + 'px');
	$("#footer").css('left', ((width - $("#footer").width() ) / 2 )  + 'px');
});

$(window).resize(function(){
	var width = $(window).width();
	var height = $(window).height();
	
	$("#homeBox").css('left', ((width - $("#homeBox").width() ) / 2 )  + 'px');
	$("#aboutBox").css('left', ((width - $("#aboutBox").width() ) / 2 )  + 'px');
	$("#options").css('left', ((width - $("#options").width() ) / 2 )  + 'px');
	$("#footer").css('left', ((width - $("#footer").width() ) / 2 )  + 'px');
});