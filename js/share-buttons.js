const shareBtns = document.querySelectorAll(".share-btn");

function onWechatBridgeReady() {
  // wechat specific callback
  let url = window.location.href; // Get current page URL
  let title = document.title; // Get current page title
  let img_url = document.querySelectorAll('meta[property="og:image"]')[0]
    .content;
  let desc = document.querySelectorAll('meta[name="description"]')[0].content;
  window.WeixinJSBridge.invoke("shareTimeline", {
    img_url: img_url,
    link: url,
    desc: "陈博远的主页: " + desc,
    title: "陈博远的主页: " + title,
  });
}

shareBtns.forEach((btn) => {
  btn.addEventListener("click", (e) => {
    e.preventDefault();
    let socialMedia = btn.className.split(" ")[1];
    let url = window.location.href; // Get current page URL
    let title = document.title; // Get current page title
    let img_url = document.querySelectorAll('meta[property="og:image"]')[0]
      .content;
    let desc = document.querySelectorAll('meta[name="description"]')[0].content;

    switch (socialMedia) {
      case "twitter":
        window.open(
          `https://twitter.com/intent/tweet?text=Boyuan%20Chen%27s%20Homepage%3A%20${title}&url=${url}`
        );
        break;
      case "wechat":
        if (typeof WeixinJSBridge == "undefined") {
          if (document.addEventListener) {
            document.addEventListener(
              "WeixinJSBridgeReady",
              onWechatBridgeReady,
              false
            );
          } else if (document.attachEvent) {
            document.attachEvent("WeixinJSBridgeReady", onWechatBridgeReady);
            document.attachEvent("onWeixinJSBridgeReady", onWechatBridgeReady);
          }
        } else {
          onWechatBridgeReady();
        }
        break;
      // case "linkedin":
      //   window.open(
      //     `https://www.linkedin.com/feed/?shareActive=true&text=Boyuan%20Chen%27s%20Homepage%3A+${title}+${url}`
      //   );
      //   break;
      // case "reddit":
      //   window.open(
      //     `https://www.reddit.com/submit?url=${url}&title=Boyuan+Chen+s+Homepage+${title}`
      //   );
      //   break;
    }
  });
});
